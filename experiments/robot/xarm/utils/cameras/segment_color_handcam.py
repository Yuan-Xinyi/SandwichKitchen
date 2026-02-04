import cv2
import numpy as np
import pyrealsense2 as rs
import yaml
import time
import json

# ---- Robot visualization ----
from wrs.robot_sim.robots.xarmlite6_wg.x6wg2 import XArmLite6WG2
from wrs.robot_con.xarm_lite6.xarm_lite6_x import XArmLite6X
import wrs.basis.robot_math as rm
import wrs.visualization.panda.world as wd
import wrs.modeling.geometric_model as mgm

def homomat_transform_points(trans: np.ndarray, pcd: np.ndarray) -> np.ndarray:
    pcd_t = np.dot(trans, pcd.T).T
    
    return pcd_t

def cam_to_w_coord(pos, rotmat, hand_to_eye_mat: np.ndarray, pcd: np.ndarray, toggle_debug=False) -> np.ndarray:
    if pcd.shape[0] == 3:
        w_coord = np.array([1.0])
        pcd = np.hstack((pcd, w_coord))
        pcd = pcd.reshape(-1, 4)
    # 4x4 Tool Center Point (TCP), describes the position and orientation of the TCP in the robot's coordinate system.
    rbt_tcp_homomat = rm.homomat_from_posrot(pos, rotmat)
    trans = np.dot(rbt_tcp_homomat, hand_to_eye_mat)
    pcd_t = np.dot(trans, pcd.T).T

    return pcd_t

# ======================================
# 颜色阈值
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
# 读取相机外参（Camera → Robot）
# ======================================
with open("experiments/robot/xarm/utils/cameras/manual_calibration.json", "r") as f:
    extrinsic = json.load(f)
extrinsic_mat = np.array(extrinsic['affine_mat'])
print("[Camera] Loaded extrinsic:\n", extrinsic_mat)
rotation = extrinsic_mat[:3, :3]
translation = extrinsic_mat[:3, 3]
print("[Camera] Rotation:\n", rotation)
print("[Camera] Translation:\n", translation)

# ======================================
# 初始化 RealSense
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

camid = 1
intr = pipelines[camid].get_active_profile().get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
print("[Camera] Intrinsics:", intr)

# ======================================
# 启动 Robot 可视化
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
rbtx.homeconf()
rbtx._gripper_x.open()
print("[Robot] Robot ready, has moved to home configuration.")

# ======================================
# 主循环（单相机模式）
# ======================================
# while True:
frames = pipelines[camid].wait_for_frames()
frames = align_to_color[camid].process(frames)

color_frame = frames.get_color_frame()
depth_frame = frames.get_depth_frame()

# if not color_frame or not depth_frame:
#     print("[WARN] Null frame detected.")
#     continue

color = np.asanyarray(color_frame.get_data())
hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)

# --------------------------------------
# 颜色 mask
# --------------------------------------
mask_red = cv2.inRange(hsv, np.array(COLOR_THRESHOLDS["RED_LOW_1"]),
                            np.array(COLOR_THRESHOLDS["RED_HIGH_1"])) \
            + cv2.inRange(hsv, np.array(COLOR_THRESHOLDS["RED_LOW_2"]),
                            np.array(COLOR_THRESHOLDS["RED_HIGH_2"]))

mask_green = cv2.inRange(hsv, np.array(COLOR_THRESHOLDS["GREEN_LOW"]),
                                np.array(COLOR_THRESHOLDS["GREEN_HIGH"]))

boxes = []

# --------------------------------------
# 找轮廓
# --------------------------------------
target_masks = [("green", mask_green)] # [("red", mask_red), ("green", mask_green)]
for color_name, mask in target_masks:
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for c in cnts:
        if cv2.contourArea(c) < 300:
            continue

        x, y, w, h = cv2.boundingRect(c)
        boxes.append((color_name, x, y, x+w, y+h))

# --------------------------------------
# 处理每个方块
# --------------------------------------
for color_name, x1, y1, x2, y2 in boxes:
    color_bgr = (0, 0, 255) if color_name == "red" else (0, 255, 0)
    cv2.rectangle(color, (x1, y1), (x2, y2), color_bgr, 2)

    u = (x1 + x2) // 2
    v = (y1 + y2) // 2
    cv2.circle(color, (u, v), 5, color_bgr, -1)

    # --------------------------------------
    # (u,v) → 相机深度
    # --------------------------------------
    depth_val = depth_frame.get_distance(u, v)

    if depth_val == 0:
        print("[WARN] Depth=0, skip.")
        continue

    camera_xyz = np.array(rs.rs2_deproject_pixel_to_point(intr, [u, v], depth_val))  # meter
    print(f"[Camera] {color_name} block at camera coords:", camera_xyz)
    # --------------------------------------
    # 相机坐标 → 机器人坐标
    # --------------------------------------
    # pred_robot_xyz = rotation @ camera_xyz + translation
    pos, rotmat = rbtx.get_pose()
    pred_robot_xyz = cam_to_w_coord(pos, rotmat, extrinsic_mat, camera_xyz).flatten()
    print(f"[Camera] {color_name} block at robot coords:", pred_robot_xyz)

    # 可视化
    pred_robot_xyz = pred_robot_xyz[:3]
    mgm.gen_frame(pos=pred_robot_xyz, rotmat=home_rot).attach_to(base)
    jnt = robot.ik(pred_robot_xyz, home_rot)

    if jnt is not None:
        robot.goto_given_conf(jnt)
        robot.gen_meshmodel(rgb=[1,0,0], alpha=0.5).attach_to(base)
        base.run()
        # base.run()
        # rbtx.move_j(jnt)
        move_xyz_top = pred_robot_xyz + np.array([0, 0, 0.03])
        rbtx.move_p(pos=move_xyz_top, rot=home_rot)
        rbtx.move_p(pos=pred_robot_xyz, rot=home_rot)

    else:
        print("[WARN] IK failed.")
        robot.goto_given_conf([-2.338697,  0.790951,  1.274774, -3.199884, -0.439821, -0.701604])
        robot.gen_meshmodel(rgb=[1,1,0], alpha=0.5).attach_to(base)

# --------------------------------------
# 显示画面
# --------------------------------------
cv2.imshow("D405 View", color)
if cv2.waitKey(1) & 0xFF == ord('q'):
    pass

pipeline.stop()
print("[Camera] Pipeline stopped.")