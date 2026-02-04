import os
import cv2
import numpy as np
import pyrealsense2 as rs
import yaml
import time
import threading

# ---- Robot visualization ----
from wrs.robot_sim.robots.xarmlite6_wg.x6wg2 import XArmLite6WG2
from wrs.robot_con.xarm_lite6.xarm_lite6_x import XArmLite6X
import wrs.basis.robot_math as rm
import wrs.visualization.panda.world as wd
import wrs.modeling.geometric_model as mgm

from data_recoder import DataRecorder

# ======================================
# camera color thresholds
# ======================================
def get_centroid_from_mask(mask):
    ys, xs = np.where(mask > 0)
    if len(xs) == 0:
        return None
    return int(np.mean(xs)), int(np.mean(ys))

COLOR_THRESHOLDS = {
    "RED_LOW_1":  [0, 150, 80],
    "RED_HIGH_1": [5, 255, 255],        
    "RED_LOW_2" : [175, 150, 80],
    "RED_HIGH_2" : [180, 255, 255],
    "GREEN_LOW":  [35, 80, 70],
    "GREEN_HIGH": [85, 255, 255]
}

# ======================================
# read camera extrinsics (Camera → Robot)
# ======================================
with open("experiments/robot/xarm/utils/cameras/camera_config.yaml", "r") as f:
    cfg_yaml = yaml.safe_load(f)
extrinsic = np.array(cfg_yaml["camera_extrinsics"]["T_camera_to_robot"])
print("[Camera] Loaded extrinsic:\n", extrinsic)

rotation = extrinsic[:3, :3]
translation = extrinsic[:3, 3]

# ======================================
# Initialize RealSense D405 cameras
# ======================================
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
time.sleep(2)  # camera warmup
print("[Camera] Two RealSense cameras ready!")
intr = pipelines[0].get_active_profile().get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
print("[Camera] Intrinsics:", intr)

# ======================================
# Start Robot visualization
# ======================================
'''simulated robot'''
base = wd.World(cam_pos=[2, 0, 1.5], lookat_pos=[0, 0, .2])
mgm.gen_frame().attach_to(base)
robot = XArmLite6WG2(pos=np.array([0, 0, 0]), enable_cc=True)
robot.goto_given_conf([-2.1713  ,  0.44797 ,  1.690495, -3.135073, -1.232058, -0.601129])
robot.gen_meshmodel(rgb=[0,1,1], alpha=0.5).attach_to(base)
home_pos, home_rot = robot.fk(robot.get_jnt_values())

'''real robot'''
rbtx = XArmLite6X(ip='192.168.1.152', has_gripper=True)
rbtx._gripper_x.open()
rbtx.homeconf()
print("[Robot] Robot ready, has moved to home configuration.")


# ======================================
# Threading Setup
# ======================================
# Queues: Collector Thread -> Recorder Thread
QUEUE_MAIN_COLLECT = rs.frame_queue(capacity=5)
QUEUE_WRIST_COLLECT = rs.frame_queue(capacity=5)

# Synchronization Events and Result Sharing
QUEUE_INFERENCE = rs.frame_queue(capacity=1) 
event_start_control = threading.Event()
event_control_done = threading.Event() 
control_results = []
lock_results = threading.Lock()

# --------------------------------------
# Data Collector Thread (Single consumer of RealSense pipelines)
# --------------------------------------
class DataCollector(threading.Thread):
    def __init__(self, pipelines, align_to_color, q_main, q_wrist):
        super().__init__()
        self.pipelines = pipelines
        self.align_to_color = align_to_color
        self.q_main = q_main
        self.q_wrist = q_wrist
        self.running = True
        self.daemon = True

    def run(self):
        print("[Collector Thread] Started.")
        while self.running:
            try:
                # Wait for main camera frames (30Hz)
                frames_main = self.pipelines[0].wait_for_frames()
                frames_main = self.align_to_color[0].process(frames_main)
                self.q_main.enqueue(frames_main)
                
                # Poll for wrist camera frames
                frames_wrist = self.pipelines[1].poll_for_frames() 
                if frames_wrist:
                    frames_wrist = self.align_to_color[1].process(frames_wrist)
                    self.q_wrist.enqueue(frames_wrist.get_color_frame())
                
            except Exception as e:
                # print(f"[Collector Thread] Error capturing frame: {e}")
                time.sleep(0.001)

# --------------------------------------
# Control Logic Thread (Inference / Robot Control)
# --------------------------------------
def control_logic(rbtx, pipelines, align_to_color, intr, rotation, translation, home_rot, queue_inference, recorder):
    
    print("[Control Thread] Started, waiting for Main Thread signal...")
    
    while True:
        event_start_control.wait() 
        
        # --- 1. Get Frameset for Inference ---
        try:
            frames = queue_inference.wait_for_frame(500) 
            frames = frames.as_frameset() 
        except RuntimeError:
            print("[Control Thread] ERROR: Failed to get frame from queue.")
            event_control_done.set()
            continue

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        
        if not color_frame or not depth_frame:
            print("[Control Thread] WARN: Null frame detected after receiving.")
            event_control_done.set()
            continue
            
        color = np.asanyarray(color_frame.get_data())
        hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
        
        # --- 2. Color Detection and Feature Extraction ---
        mask_red = cv2.inRange(hsv, np.array(COLOR_THRESHOLDS["RED_LOW_1"]),
                                    np.array(COLOR_THRESHOLDS["RED_HIGH_1"])) \
                    + cv2.inRange(hsv, np.array(COLOR_THRESHOLDS["RED_LOW_2"]),
                                    np.array(COLOR_THRESHOLDS["RED_HIGH_2"]))
        mask_green = cv2.inRange(hsv, np.array(COLOR_THRESHOLDS["GREEN_LOW"]),
                                        np.array(COLOR_THRESHOLDS["GREEN_HIGH"]))

        boxes = []
        target_masks = [("red", mask_red), ("green", mask_green)] 
        for color_name, mask in target_masks:
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                if cv2.contourArea(c) < 300:
                    continue
                x, y, w, h = cv2.boundingRect(c)
                boxes.append((color_name, x, y, x+w, y+h))
        
        # --- 3. Process Each Block (Single Execution) ---
        success = False
        for color_name, x1, y1, x2, y2 in boxes:
            
            print('#---------------------------------------------------#')
            print(f"[Camera] Detected {color_name} block at pixel coords:", (x1, y1, x2, y2))
            print('#---------------------------------------------------#')

            u, v = (x1 + x2) // 2, (y1 + y2) // 2
            
            # Pixel to Camera Depth
            depth_val = depth_frame.get_distance(u, v)

            if depth_val == 0:
                print("[WARN] Depth=0, skip.")
                continue

            camera_xyz = np.array(rs.rs2_deproject_pixel_to_point(intr, [u, v], depth_val))
            pred_robot_xyz = rotation @ camera_xyz + translation
            print(f"[Camera] {color_name} block at robot coords:", pred_robot_xyz)
            
            # Robot movement
            pred_robot_xyz[2] = 0.0025
            rbtx.move_p(pos=pred_robot_xyz, rot=home_rot)
            
            # Wrist camera adjustment logic
            if color_name != "red":
                '''insert align two edges here'''
                # Read latest frame from recorder cache for adjustment
                with recorder.lock:
                    if recorder.last_color_frames[1] is not None:
                        img2 = recorder.last_color_frames[1].copy()
                    else:
                        continue 
                
                hsv = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV)

                mask_r = cv2.inRange(hsv, np.array(COLOR_THRESHOLDS["RED_LOW_1"]), np.array(COLOR_THRESHOLDS["RED_HIGH_1"])) | cv2.inRange(hsv, np.array(COLOR_THRESHOLDS["RED_LOW_2"]), np.array(COLOR_THRESHOLDS["RED_HIGH_2"]))
                mask_g = cv2.inRange(hsv, np.array(COLOR_THRESHOLDS["GREEN_LOW"]), np.array(COLOR_THRESHOLDS["GREEN_HIGH"]))

                # compute top edges
                red_top  = np.min(np.where(mask_r>0)[0])
                green_top = np.min(np.where(mask_g>0)[0])

                dy = green_top - red_top             # pixel offset to align top edges
                pred_robot_xyz[1] += dy * 0.00012    # move robot (scale example)
                rbtx.move_p(pos=pred_robot_xyz, rot=home_rot)
                rbtx._gripper_x.open()
                rbtx.homeconf()
                success = True
                break
            
            else: # Red block adjustment logic
                '''adjustment based on wrist camera'''
                # Read latest frame from recorder cache for adjustment
                with recorder.lock:
                    if recorder.last_color_frames[1] is not None:
                        img = recorder.last_color_frames[1].copy()
                    else:
                        continue 

                # convert to HSV
                hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                mask1 = cv2.inRange(hsv, np.array(COLOR_THRESHOLDS["RED_LOW_1"]), np.array(COLOR_THRESHOLDS["RED_HIGH_1"]))
                mask2 = cv2.inRange(hsv, np.array(COLOR_THRESHOLDS["RED_LOW_2"]), np.array(COLOR_THRESHOLDS["RED_HIGH_2"]))
                mask = mask1 | mask2

                # centroid of the color mask
                ys, xs = np.where(mask > 0)
                if len(xs) == 0:
                    print("no red detected")
                    continue
                cx, cy = int(xs.mean()), int(ys.mean())

                # target grasp position
                dx = cx - 707
                dy = cy - 515
                print(f"Current center: {cx}, {cy}. you need to shift: dx={dx}, dy={dy}")
                x_scale = 0.00012 
                y_scale = 0.00012
                pred_robot_xyz[0] -= dx*x_scale
                pred_robot_xyz[1] += dy*y_scale
                print("new target:", pred_robot_xyz)
                
                rbtx.move_p(pos=pred_robot_xyz, rot=home_rot)
                pred_robot_xyz[2] = -0.02335815
                rbtx.move_p(pos=pred_robot_xyz, rot=home_rot)
                rbtx._gripper_x.set(0.6)
                success = True
                continue

        # --- 4. Notify Main Thread ---
        with lock_results:
            control_results.append({"status": "success" if success else "fail"})

        event_start_control.clear() 
        event_control_done.set()    


# ======================================
# Main execution
# ======================================
if __name__ == '__main__':
    
    # --------------------------------------
    # Start Asynchronous Threads
    # --------------------------------------
    
    # 1. Start Data Collector Thread
    collector_thread = DataCollector(pipelines, align_to_color, QUEUE_MAIN_COLLECT, QUEUE_WRIST_COLLECT)
    collector_thread.start()
    time.sleep(0.5) 

    # 2. Start Data Recorder Thread
    recorder = DataRecorder(rbtx, QUEUE_MAIN_COLLECT, QUEUE_WRIST_COLLECT, fps=20) 
    recorder.start()
    time.sleep(1) 

    # 3. Start Control Thread
    control_thread = threading.Thread(target=control_logic, args=(
        rbtx, pipelines, align_to_color, intr, rotation, translation, home_rot, 
        QUEUE_INFERENCE, recorder # Pass recorder instance to control thread for data access
    ))
    control_thread.daemon = True
    control_thread.start()

    # --------------------------------------
    # Main Thread: Trigger Single Task
    # --------------------------------------
    
    should_start_control = True

    try:
        if should_start_control:
            print("[Main Thread] Triggering control task...")
            
            # 1. Get the latest frameset from the collector queue and push to inference queue
            try:
                frameset_for_inference = QUEUE_MAIN_COLLECT.wait_for_frame(500)
                QUEUE_INFERENCE.enqueue(frameset_for_inference)
            except RuntimeError:
                print("[Main Thread] WARN: Failed to get frame for initial inference.")
                
            # 2. Trigger and wait (Recorder continues running in background)
            event_start_control.set() 
            event_control_done.wait() 
            
            # 3. Receive results
            with lock_results:
                if control_results:
                    result = control_results.pop(0)
                    print(f"[Main Thread] Control finished. Result: {result}")
            
            # Cleanup state
            event_start_control.clear()
            event_control_done.clear()
            
    except KeyboardInterrupt:
        print("\n[Main Thread] Exiting due to keyboard interrupt.")

    # --------------------------------------
    # Stop and Save
    # --------------------------------------
    collector_thread.running = False
    recorder.running = False

    for p in pipelines:
        p.stop()
    print("[Camera] Pipelines stopped.")

    collector_thread.join()
    recorder.join()

    name = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    demo_20hz_record = f"experiments/robot/xarm/data/stack_r2g_{name}"
    os.makedirs(os.path.dirname(demo_20hz_record), exist_ok=True)
    
    print(f"[Recorder] Recorded {len(recorder.data)} samples, saving to {demo_20hz_record}.h5")
    recorder.save(f"{demo_20hz_record}.h5")