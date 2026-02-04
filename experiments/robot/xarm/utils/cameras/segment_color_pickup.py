import cv2
import numpy as np
import pyrealsense2 as rs
import yaml
import time

# ---- Robot visualization ----
from wrs.robot_sim.robots.xarmlite6_wg.x6wg2 import XArmLite6WG2
from wrs.robot_con.xarm_lite6.xarm_lite6_x import XArmLite6X
import wrs.basis.robot_math as rm
import wrs.visualization.panda.world as wd
import wrs.modeling.geometric_model as mgm

def get_centroid_from_mask(mask):
    ys, xs = np.where(mask > 0)
    if len(xs) == 0:
        return None
    return int(np.mean(xs)), int(np.mean(ys))

# ======================================
# color thresholds
# ======================================
COLOR_THRESHOLDS = {
    "RED_LOW_1":  [0, 150, 80],
    "RED_HIGH_1": [5, 255, 255],        
    "RED_LOW_2" : [175, 150, 80],
    "RED_HIGH_2" : [180, 255, 255],
    "GREEN_LOW":  [35, 80, 70],
    "GREEN_HIGH": [85, 255, 255]
}

# ======================================
# load camera extrinsics (Camera → Robot)
# ======================================
with open("experiments/robot/xarm/utils/cameras/camera_config.yaml", "r") as f:
    cfg_yaml = yaml.safe_load(f)
extrinsic = np.array(cfg_yaml["camera_extrinsics"]["T_camera_to_robot"])
print("[Camera] Loaded extrinsic:\n", extrinsic)

rotation = extrinsic[:3, :3]
translation = extrinsic[:3, 3]

# ======================================
# initialize RealSense
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
print("[Camera] Two RealSense cameras ready!")
intr = pipelines[0].get_active_profile().get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
print("[Camera] Intrinsics:", intr)

# ======================================
# start Robot visualization
# ======================================
'''simulated robot'''
base = wd.World(cam_pos=[2, 0, 1.5], lookat_pos=[0, 0, .2])
mgm.gen_frame().attach_to(base)
robot = XArmLite6WG2(pos=np.array([0, 0, 0]), enable_cc=True)
robot.goto_given_conf([-2.1713  ,  0.44797 ,  1.690495, -3.135073, -1.232058, -0.601129])
robot.gen_meshmodel(rgb=[0,1,1], alpha=0.5).attach_to(base)
home_pos, home_rot = robot.fk(robot.get_jnt_values())
print("[Robot] Home pos:", home_pos)
print("[Robot] Home rot:\n", home_rot)
mgm.gen_frame(pos=home_pos, rotmat=home_rot).attach_to(base)
# base.run()

'''real robot'''
rbtx = XArmLite6X(ip='192.168.1.152', has_gripper=True)
rbtx._gripper_x.open()
rbtx.homeconf()
print("[Robot] Robot ready, has moved to home configuration.")

# ======================================
# main loop (single camera mode)
# ======================================

frames = pipelines[0].wait_for_frames()
frames = align_to_color[0].process(frames)

color_frame = frames.get_color_frame()
depth_frame = frames.get_depth_frame()

if not color_frame or not depth_frame:
    print("[WARN] Null frame detected.")

color = np.asanyarray(color_frame.get_data())
hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)

# --------------------------------------
# color mask
# --------------------------------------
mask_red = cv2.inRange(hsv, np.array(COLOR_THRESHOLDS["RED_LOW_1"]),
                            np.array(COLOR_THRESHOLDS["RED_HIGH_1"])) \
            + cv2.inRange(hsv, np.array(COLOR_THRESHOLDS["RED_LOW_2"]),
                            np.array(COLOR_THRESHOLDS["RED_HIGH_2"]))

mask_green = cv2.inRange(hsv, np.array(COLOR_THRESHOLDS["GREEN_LOW"]),
                                np.array(COLOR_THRESHOLDS["GREEN_HIGH"]))

boxes = []

# --------------------------------------
# find contours
# --------------------------------------
target_masks = [("red", mask_red), ("green", mask_green)] # [("red", mask_red), ("green", mask_green)]
for color_name, mask in target_masks:
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for c in cnts:
        if cv2.contourArea(c) < 300:
            continue

        x, y, w, h = cv2.boundingRect(c)
        boxes.append((color_name, x, y, x+w, y+h))

# --------------------------------------
# process each box
# --------------------------------------
for color_name, x1, y1, x2, y2 in boxes:
    print('#---------------------------------------------------#')
    print(f"[Camera] Detected {color_name} block at pixel coords:", (x1, y1, x2, y2))
    print('#---------------------------------------------------#')
    color_bgr = (0, 0, 255) if color_name == "red" else (0, 255, 0)
    cv2.rectangle(color, (x1, y1), (x2, y2), color_bgr, 2)

    u = (x1 + x2) // 2
    v = (y1 + y2) // 2
    cv2.circle(color, (u, v), 5, color_bgr, -1)

    # --------------------------------------
    # (u,v) → camera depth
    # --------------------------------------
    depth_val = depth_frame.get_distance(u, v)

    if depth_val == 0:
        print("[WARN] Depth=0, skip.")
        continue

    camera_xyz = np.array(rs.rs2_deproject_pixel_to_point(intr, [u, v], depth_val))

    # --------------------------------------
    # camera coords → robot coords
    # --------------------------------------
    pred_robot_xyz = rotation @ camera_xyz + translation
    print(f"[Camera] {color_name} block at robot coords:", pred_robot_xyz)

    # visualization
    mgm.gen_frame(pos=pred_robot_xyz, rotmat=home_rot).attach_to(base)
    jnt = robot.ik(pred_robot_xyz, home_rot)

    if jnt is not None:
        robot.goto_given_conf(jnt)
        robot.gen_meshmodel(rgb=[1,0,0], alpha=0.5).attach_to(base)

    else:
        print("[WARN] IK failed.")
        robot.goto_given_conf([-2.338697,  0.790951,  1.274774, -3.199884, -0.439821, -0.701604])
        robot.gen_meshmodel(rgb=[1,1,0], alpha=0.5).attach_to(base)

    pred_robot_xyz[2] = 0.0025
    rbtx.move_p(pos=pred_robot_xyz, rot=home_rot)

    if color_name != "red":
        '''insert align two edges here'''
        # ---- capture a wrist image ----
        top_view = pipelines[1].wait_for_frames()
        color_frame = align_to_color[1].process(top_view).get_color_frame()
        img2 = np.asanyarray(color_frame.get_data())
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
        break

    '''adjustment based on wrist camera'''
    top_view = pipelines[1].wait_for_frames()
    top_frames = align_to_color[1].process(top_view)
    color_frame = top_frames.get_color_frame()
    img = np.asanyarray(color_frame.get_data())

    # --- convert to HSV ---
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, np.array(COLOR_THRESHOLDS["RED_LOW_1"]), np.array(COLOR_THRESHOLDS["RED_HIGH_1"]))
    mask2 = cv2.inRange(hsv, np.array(COLOR_THRESHOLDS["RED_LOW_2"]), np.array(COLOR_THRESHOLDS["RED_HIGH_2"]))
    mask = mask1 | mask2

    # --- centroid of the color mask ---
    ys, xs = np.where(mask > 0)
    if len(xs) == 0:
        print("no red detected")
        continue
    cx, cy = int(xs.mean()), int(ys.mean())

    # --- target grasp position ---
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
    
    
    # base.run()

# --------------------------------------
# display image
# --------------------------------------
cv2.imshow("D405 View", color)
if cv2.waitKey(1) & 0xFF == ord('q'):
    pass

pipeline.stop()
print("[Camera] Pipeline stopped.")