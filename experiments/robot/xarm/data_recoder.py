import time
import numpy as np
import cv2
import h5py
import threading
import pyrealsense2 as rs

class DataRecorder(threading.Thread):
    """
    Asynchronous 20Hz recorder. Reads frames from queues, deep copies data, 
    records robot state, and caches the latest frames for main thread access.
    """
    def __init__(self, rbtx, frame_queue_main, frame_queue_wrist, fps=20):
        super().__init__()
        self.rbtx = rbtx
        self.interval = 1.0 / fps
        self.data = []
        self.running = False
        self.lock = threading.Lock()
        
        self.q_main = frame_queue_main
        self.q_wrist = frame_queue_wrist
        self.n_cam = 2 

        # Thread-safe cache for the latest NumPy arrays
        self.last_color_frames = [None] * self.n_cam 
        self.last_depth_frames = [None] * self.n_cam 

    def run(self):
        self.running = True
        next_t = time.time()
        print("[Recorder] Thread started.")

        while self.running:
            now = time.time()
            if now < next_t:
                time.sleep(next_t - now)
            next_t += self.interval

            timestamp = time.time()
            
            color_data_list = []
            depth_data_list = []
            
            # --- 1. Read Frameset from Main Camera Queue ---
            try:
                # Wait briefly for a frameset from the collector thread
                fs_main = self.q_main.wait_for_frame(int(self.interval * 1000)) 
                fs_main = fs_main.as_frameset()
                color_frame_main = fs_main.get_color_frame()
                depth_frame_main = fs_main.get_depth_frame()
                
                # Deep copy data to NumPy arrays
                color_data_main = np.asanyarray(color_frame_main.get_data()).copy()
                depth_data_main = np.asanyarray(depth_frame_main.get_data()).copy()
                color_data_list.append(color_data_main)
                depth_data_list.append(depth_data_main)
                
            except RuntimeError:
                # Timeout, skip this sample
                continue 

            # --- 2. Read Color Frame from Wrist Camera Queue ---
            try:
                # Non-blocking read (poll_for_frame)
                color_frame_wrist = self.q_wrist.poll_for_frame() 
                if color_frame_wrist:
                    color_data_wrist = np.asanyarray(color_frame_wrist.get_data()).copy()
                else:
                    # Use the previously cached frame if available
                    color_data_wrist = self.last_color_frames[1].copy() if self.last_color_frames[1] is not None else np.zeros_like(color_data_main)
            
            except Exception:
                 color_data_wrist = np.zeros_like(color_data_main)

            color_data_list.append(color_data_wrist)
            depth_data_list.append(None) # Wrist camera doesn't provide depth

            # Update thread-safe cache
            with self.lock:
                self.last_color_frames = color_data_list
                self.last_depth_frames = depth_data_list

            # ---- Robot state & save record ----
            jnt = self.rbtx.get_jnt_values()
            ee_pos, ee_rot = self.rbtx.get_pose()
            # No serial reading, avoid conflict
            grip = float(getattr(self.rbtx._gripper_x, "last_position", 0.0))

            self.data.append({
                "timestamp": timestamp,
                "color_frames": color_data_list,   
                "depth_frames": depth_data_list,   
                "joints": jnt.tolist(),
                "ee_pos": ee_pos.tolist(),
                "ee_rot": ee_rot.tolist(),
                "gripper": grip
            })

        print("[Recorder] Thread stopped.")

    def stop(self):
        self.running = False

    def save(self, path, image_compression="gzip"):
        data = self.data
        N = len(data)
        if N == 0:
            print("[HDF5] No data to save.")
            return

        n_cam = len(data[0]["color_frames"])
        H, W, C = data[0]["color_frames"][0].shape

        print(f"[HDF5] Saving {N} samples, {n_cam} cameras...")

        # Prepare color frames
        all_color = []
        for cam_id in range(n_cam):
            frames_cam = []
            for d in data:
                bgr = d["color_frames"][cam_id]
                rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                frames_cam.append(rgb)
            all_color.append(np.stack(frames_cam, axis=0))

        # Prepare depth frames
        all_depth = []
        for cam_id in range(n_cam):
            depth_cam = []
            for d in data:
                depth_data = d["depth_frames"][cam_id]
                if depth_data is not None:
                    depth_cam.append(depth_data)
                else:
                    # Pad with zero matrix to maintain stack dimension
                    depth_cam.append(np.zeros((H, W), dtype=np.uint16)) 
            all_depth.append(np.stack(depth_cam, axis=0))

        jnts  = np.array([d["joints"] for d in data])
        ee_pos = np.array([d["ee_pos"] for d in data])
        ee_rot = np.array([d["ee_rot"] for d in data])
        grip = np.array([d["gripper"] for d in data])
        timestamps = np.array([d["timestamp"] for d in data])

        # ---- write HDF5 ----
        with h5py.File(path, "w") as f:
            f.create_dataset("timestamps", data=timestamps, compression="gzip")

            # camera groups
            for cam_id in range(n_cam):
                grp = f.create_group(f"cam{cam_id}")
                grp.create_dataset("color", data=all_color[cam_id],
                                   compression=image_compression)
                # Only save depth if data exists
                if all_depth[cam_id].sum() > 0:
                    grp.create_dataset("depth", data=all_depth[cam_id],
                                       compression="gzip")

            # robot group
            robot = f.create_group("robot")
            robot.create_dataset("joints", data=jnts, compression="gzip")
            robot.create_dataset("ee_pos", data=ee_pos, compression="gzip")
            robot.create_dataset("ee_rot", data=ee_rot, compression="gzip")
            robot.create_dataset("gripper", data=grip, compression="gzip")

            f.attrs["n_samples"] = N
            f.attrs["n_cameras"] = n_cam
            f.attrs["image_shape"] = (H, W, C)

        print(f"[HDF5] Saved -> {path}")