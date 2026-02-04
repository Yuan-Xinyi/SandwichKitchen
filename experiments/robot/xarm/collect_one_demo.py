import time
import numpy as np
import yaml
import cv2
import threading
import wrs.basis.robot_math as rm

def move_from_top(rbtx, pos, rot, dz=0.10):
    approach_pos = [pos[0], pos[1], pos[2] + dz]
    rbtx.move_p(pos=approach_pos, rot=rot)
    rbtx.move_p(pos=pos, rot=rot)

# --- init robot ---
from wrs.robot_con.xarm_lite6.xarm_lite6_x import XArmLite6X
rbtx = XArmLite6X(ip='192.168.1.152', has_gripper=True)
rbtx.homeconf()
rbtx._gripper_x.open()
print("[Main] Robot initialized and moved to home configuration.")

# ====================================================
# Multithreaded Recorder: 20 Hz camera + robot state collection
# ====================================================
class RecorderThread(threading.Thread):
    def __init__(self, rbtx, pipelines):
        super().__init__()
        self.rbtx = rbtx
        self.pipelines = pipelines      # <-- now using pipelines
        self.interval = 1.0 / 20.0
        self.data = []
        self.running = False

    def run(self):
        self.running = True
        next_t = time.time()

        print("[Recorder] Thread started (20Hz).")

        while self.running:
            now = time.time()
            if now < next_t:
                time.sleep(next_t - now)
            timestamp = time.time()

            # --- get image from pipelines ---
            frames = []
            for pipe in self.pipelines:
                fs = pipe.wait_for_frames()
                color_frame = fs.get_color_frame()
                if not color_frame:
                    img = np.zeros((480, 640, 3), dtype=np.uint8)
                else:
                    img = np.asanyarray(color_frame.get_data())
                    img = cv2.resize(img, (640,480))
                frames.append(img)

            # --- robot state ---
            jnt = self.rbtx.get_jnt_values()
            ee_pos, ee_rot = self.rbtx.get_pose()
            grip = self.rbtx._gripper_x.last_position

            # --- save record ---
            self.data.append({
                "timestamp": timestamp,
                "frames": frames,
                "joints": jnt.tolist(),
                "ee_pos": ee_pos.tolist(),
                "ee_rot": ee_rot.tolist(),
                "gripper": float(grip)
            })

            next_t += self.interval

        print("[Recorder] Thread stopped.")

    def stop(self):
        self.running = False


import h5py
import numpy as np

def save_to_hdf5(path, data, image_compression="gzip"):
    N = len(data)
    if N == 0:
        print("[HDF5] No data to save.")
        return

    n_cam = len(data[0]["frames"])
    H, W, C = data[0]["frames"][0].shape

    print(f"[HDF5] Saving {N} samples, {n_cam} cameras, image size {H}x{W} ...")

    all_frames = []
    for cam_id in range(n_cam):
        frames_cam = []

        for d in data:
            bgr = d["frames"][cam_id]              # RealSense BGR
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)  # change to RGB
            frames_cam.append(rgb)

        frames_cam = np.stack(frames_cam, axis=0)
        all_frames.append(frames_cam)

    jnts  = np.array([d["joints"] for d in data])
    ee_pos = np.array([d["ee_pos"] for d in data])
    ee_rot = np.array([d["ee_rot"] for d in data])
    grip = np.array([d["gripper"] for d in data])
    timestamps = np.array([d["timestamp"] for d in data])

    with h5py.File(path, "w") as f:
        f.create_dataset("timestamps", data=timestamps, compression="gzip")

        # cameras
        for cam_id in range(n_cam):
            grp = f.create_group(f"cam{cam_id}")
            grp.create_dataset(
                "frames",
                data=all_frames[cam_id],
                compression=image_compression
            )

        # robot
        robot = f.create_group("robot")
        robot.create_dataset("joints", data=jnts, compression="gzip")
        robot.create_dataset("ee_pos", data=ee_pos, compression="gzip")
        robot.create_dataset("ee_rot", data=ee_rot, compression="gzip")
        robot.create_dataset("gripper", data=grip, compression="gzip")

        f.attrs["n_samples"] = N
        f.attrs["n_cameras"] = n_cam
        f.attrs["image_shape"] = (H, W, C)

    print(f"[HDF5] Saved -> {path}")



# ================================
# Show camera window before demo
# ================================
import pyrealsense2 as rs
ctx = rs.context()
devices = ctx.query_devices()
serials = [d.get_info(rs.camera_info.serial_number) for d in devices]
print("[Camera] Found devices:", serials)

pipelines = []
configs = []

for serial in serials:
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(serial)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    pipeline.start(config)
    pipelines.append(pipeline)
    configs.append(config)
align_to_color = [rs.align(rs.stream.color) for _ in serials]
print("[Camera] Two RealSense cameras ready!")
intr = pipelines[0].get_active_profile().get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
print("[Camera] Intrinsics:", intr)
time.sleep(2)  # camera warmup
print("[Main] Cameras validation finished.")

# ====================================================
# Start 20Hz recording thread (background running)
# ====================================================
recorder = RecorderThread(rbtx, pipelines)
recorder.start()

time.sleep(1)  # recorder warmup


# --- start the first demo collection ---
config = yaml.safe_load(open("experiments/robot/xarm/configs/one_demo.yaml", "r"))
sequences = ["red", "green"]
# sequences = ["red", "green", "grey"]

for id, seq in enumerate(sequences[:-1]):
    pose = config[f"{seq}_pose"]
    print(f"[Main] Moving to {seq} position: {pose}")
    move_from_top(rbtx, pos=pose[:3], rot=pose[3:])
    rbtx._gripper_x.set(0.41)
    time.sleep(0.5)

    target_pose = config[f"{sequences[-1]}_pose"]
    target_pose[2] += id * 0.02
    move_from_top(rbtx, pos=target_pose[:3], rot=target_pose[3:])
    rbtx._gripper_x.open()
    time.sleep(0.5)

# ====================================================
# stop and save recording
# ====================================================
recorder.stop()
recorder.join()

print(f"[Main] Recorded {len(recorder.data)} samples.")
save_to_hdf5("demo_20hz_record.h5", recorder.data)
