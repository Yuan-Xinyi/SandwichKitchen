import sys
import os
import numpy as np
import time

# Add the project root to sys.path to allow importing wrs and experiments
# Assuming this file is in VLA-Adapter/VLA-Adapter/mcp_control/
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if project_root not in sys.path:
    sys.path.append(project_root)

from mcp.server.fastmcp import FastMCP
from wrs.robot_con.xarm_lite6.xarm_lite6_x import XArmLite6X
import wrs.basis.robot_math as rm


# Initialize MCP Server
mcp = FastMCP("XArm Control")

# Global robot instance
robot = None
ROBOT_IP = "192.168.1.152"


'''
Robot Control Tools
'''
def get_robot():
    """Lazy initialization of the robot connection."""
    global robot
    if robot is None:
        try:
            print(f"Connecting to robot at {ROBOT_IP}...")
            # has_gripper=False for simplicity, change to True if needed
            robot = XArmLite6X(ip=ROBOT_IP, has_gripper=True)
            print("Robot connected successfully.")
        except Exception as e:
            print(f"Failed to connect to robot: {e}")
            raise e
    return robot


@mcp.tool()
# obtain current joint angles
def get_joint_angles() -> list[float]:
    """
    Get the current joint angles of the XArm robot.
    Returns a list of 6 float values representing the joint angles in radians.
    """
    try:
        rbt = get_robot()
        jnt_values = rbt.get_jnt_values()
        # Convert numpy array to list for JSON serialization
        return jnt_values.tolist()
    except Exception as e:
        return f"Error getting joint angles: {str(e)}"


@mcp.tool()
# obtain current end-effector pose
def get_pose() -> dict:
    """
    Get the current end-effector position and rotation matrix (using FK).
    Returns a dict: {"position": [x, y, z], "rotation": [[...],[...],[...]]}
    """
    try:
        rbt = get_robot()
        return rbt.get_pose()
    except Exception as e:
        return {"error": str(e)}


# @mcp.tool()
# # move to home configuration
# def homeconf() -> str:
#     """
#     Move the robot to its home configuration.
#     Returns 'ok' if successful, or error message.
#     """
#     try:
#         rbt = get_robot()
#         rbt.homeconf()
#         return "[INFO] robot has moved to home configuration."
#     except Exception as e:
#         return f"Error moving to home configuration: {str(e)}"


# @mcp.tool()
# # move to given joint angles
# def move_j(joint_angles: list[float]) -> str:
#     """
#     Move the robot to the specified joint angles (in radians).
#     joint_angles: list of 6 floats.
#     Returns 'ok' if successful, or error message.
#     """
#     try:
#         rbt = get_robot()
#         rbt.move_j(np.array(joint_angles))
#         return f"[INFO] robot has moved to joint angles {joint_angles}."
#     except Exception as e:
#         return f"Error moving to joint angles: {str(e)}"
    
# # move to given end-effector pose
# @mcp.tool()
# def move_p(position: list[float], rotation: list[list[float]]) -> str:
#     """
#     Move the robot to the specified end-effector pose.
#     position: [x, y, z], rotation: 3x3 matrix (list of lists)
#     Returns 'ok' if successful, or error message.
#     """
#     try:
#         rbt = get_robot()
#         rbt.move_p(np.array(position), np.array(rotation))
#         return f"[INFO] robot has moved to position {position} with rotation {rotation}."
#     except Exception as e:
#         return f"Error moving to position: {str(e)}"
    

'''
vision tools
'''
import cv2
import pyrealsense2 as rs
import json


# campture the real time image from the specified camera and save to local file
def get_and_save_camera_image(cam_id: int = 1, save_path: str = None) -> str:
    """
    获取指定相机的当前彩色图像并保存为本地文件。
    Args:
        cam_id: 相机编号。0=侧视角，1=手眼相机。
        save_path: 保存路径（如未指定则自动命名）。
    Returns:
        保存的文件路径或错误信息。
    """
    import cv2
    import pyrealsense2 as rs
    import datetime
    ctx = rs.context()
    devices = ctx.query_devices()
    serials = [d.get_info(rs.camera_info.serial_number) for d in devices]
    if cam_id >= len(serials):
        return f"Camera id {cam_id} not found. Found {len(serials)} cameras."
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(serials[cam_id])
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    pipeline.start(config)
    try:
        time.sleep(2)
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return "No color frame."
        color = np.asanyarray(color_frame.get_data())
        if save_path is None:
            now = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            save_path = f"camera{cam_id}_{now}.jpg"
        cv2.imwrite(save_path, color)
        return save_path
    except Exception as e:
        return str(e)
    finally:
        pipeline.stop()


# Detect colored blocks and return their robot coordinates
def detect_colored_blocks_and_transform(calib_json_path: str = "/home/lqin/VLA-Adapter/VLA-Adapter/experiments/robot/xarm/utils/cameras/manual_calibration.json") -> list:
    """
    Detect green/red blocks in the hand camera image and return their robot coordinates.
    Returns a list of dicts: [{"color": str, "robot_xyz": [x, y, z]}]
    """
    rbt = get_robot()
    rbt.homeconf()
    def _cam_to_w_coord(pos, rotmat, hand_to_eye_mat: np.ndarray, pcd: np.ndarray, toggle_debug=False) -> np.ndarray:
        if pcd.shape[0] == 3:
            w_coord = np.array([1.0])
            pcd = np.hstack((pcd, w_coord))
            pcd = pcd.reshape(-1, 4)
        # 4x4 Tool Center Point (TCP), describes the position and orientation of the TCP in the robot's coordinate system.
        rbt_tcp_homomat = rm.homomat_from_posrot(pos, rotmat)
        trans = np.dot(rbt_tcp_homomat, hand_to_eye_mat)
        pcd_t = np.dot(trans, pcd.T).T

        return pcd_t

    cam_id = 1
    COLOR_THRESHOLDS = {
        "RED_LOW_1":  [0, 150, 80],
        "RED_HIGH_1": [5, 255, 255],        
        "RED_LOW_2" : [175, 150, 80],
        "RED_HIGH_2" : [180, 255, 255],
        "GREEN_LOW":  [35, 80, 70],
        "GREEN_HIGH": [85, 255, 255]
    }

    with open(calib_json_path, "r") as f:
        extrinsic = json.load(f)
    extrinsic_mat = np.array(extrinsic['affine_mat'])
    rotation = extrinsic_mat[:3, :3]
    translation = extrinsic_mat[:3, 3]

    # Initialize RealSense camera
    ctx = rs.context()
    devices = ctx.query_devices()
    serials = [d.get_info(rs.camera_info.serial_number) for d in devices]
    if cam_id >= len(serials):
        return [{"error": f"Camera id {cam_id} not found. Found {len(serials)} cameras."}]
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(serials[cam_id])
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    pipeline.start(config)
    align = rs.align(rs.stream.color)
    try:
        time.sleep(2)
        frames = pipeline.wait_for_frames()
        frames = align.process(frames)
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            return [{"error": "No color or depth frame."}]
        color = np.asanyarray(color_frame.get_data())
        hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
        mask_red = cv2.inRange(hsv, np.array(COLOR_THRESHOLDS["RED_LOW_1"]), np.array(COLOR_THRESHOLDS["RED_HIGH_1"])) \
                 + cv2.inRange(hsv, np.array(COLOR_THRESHOLDS["RED_LOW_2"]), np.array(COLOR_THRESHOLDS["RED_HIGH_2"]))
        mask_green = cv2.inRange(hsv, np.array(COLOR_THRESHOLDS["GREEN_LOW"]), np.array(COLOR_THRESHOLDS["GREEN_HIGH"]))
        results = []
        for color_name, mask in [("red", mask_red), ("green", mask_green)]:
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                if cv2.contourArea(c) < 300:
                    continue
                x, y, w, h = cv2.boundingRect(c)
                u = (x + x + w) // 2
                v = (y + y + h) // 2
                depth_val = depth_frame.get_distance(u, v)
                if depth_val == 0:
                    continue
                intr = color_frame.profile.as_video_stream_profile().get_intrinsics()
                camera_xyz = np.array(rs.rs2_deproject_pixel_to_point(intr, [u, v], depth_val))
                pos, rotmat = rbt.get_pose()
                pred_robot_xyz = _cam_to_w_coord(pos, rotmat, extrinsic_mat, camera_xyz).flatten()
                pred_robot_xyz = pred_robot_xyz[:3]
                print(f"[Camera] {color_name} block at camera coords:", camera_xyz)
                print(f"[Camera] {color_name} block at robot coords:", pred_robot_xyz)
                results.append({"color": color_name, "camera_xyz": camera_xyz.tolist(), 
                                "robot_xyz": pred_robot_xyz.tolist()})
        return results
    except Exception as e:
        return [{"error": str(e)}]
    finally:
        pipeline.stop()

def grasp(target_xyz: np.ndarray, rotmat: np.ndarray, approach_height: float, rbt) -> None:
    """Move the robot to grasp the target at target_xyz with given rotmat."""
    move_xyz_top = target_xyz + np.array([0, 0, approach_height])
    rbt.move_p(move_xyz_top, rotmat)
    target_xyz[2] = -0.03124292
    rbt.move_p(target_xyz, rotmat)
    rbt._gripper_x.set(0.6)
    

@mcp.tool()
def detect_and_grasp_colored_block(
    color: str = "green",
    cam_id: int = 1,
    calib_json_path: str = "/home/lqin/VLA-Adapter/VLA-Adapter/experiments/robot/xarm/utils/cameras/manual_calibration.json",
    approach_height: float = 0.05
) -> str:
    """
    检测指定颜色方块并抓取给定的颜色方块。
    Args:
        color: "green" 或 "red"
        cam_id: 相机编号（0=侧视角，1=手眼相机）
        calib_json_path: 手眼标定文件路径
        approach_height: 到色块上方的高度（米）
    Returns:
        操作结果字符串。
    """
    # 复用检测函数
    rbt = get_robot()
    results = detect_colored_blocks_and_transform(calib_json_path=calib_json_path)
    # 过滤目标色块
    targets = [r for r in results if r.get("color") == color and "robot_xyz" in r]
    if not targets:
        return f"No {color} block detected."

    target_xyz = np.array(targets[0]["robot_xyz"])
    _, rotmat = rbt.get_pose()
    
    try:
        grasp(target_xyz, rotmat, approach_height, rbt)
        return f"Moved to {color} block at {target_xyz.tolist()}"
    except Exception as e:
        return f"Move failed: {str(e)}"

if __name__ == "__main__":
    print("Starting MCP Server...")
    mcp.run()
