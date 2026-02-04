import cv2
import numpy as np
import pyrealsense2 as rs
import json
from pathlib import Path

json_file = Path("experiments/robot/xarm/utils/cameras/calibrate_camera.jsonl")
# json_file = Path("experiments/robot/xarm/utils/cameras/test.jsonl")

# ===== calibration  =====
PATTERN_SIZE = (4, 4)   # inner corners
SQUARE_SIZE = 0.025      # each square 2.5cm

# ===== RealSense initialization =====
if not json_file.exists():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    profile = pipeline.start(config)

    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

    print("[Info] Intrinsics:", intr)

    robot_points = []
    camera_points = []

    current_idx = 0  # current idx to collect

    print("====================================")
    print("[Press n] Select next corner and save current corner position")
    print("[Press q] Quit")
    print("====================================")

    while True:
        frames = pipeline.wait_for_frames()
        color = np.asanyarray(frames.get_color_frame().get_data())
        depth = frames.get_depth_frame()
        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)

        found, corners = cv2.findChessboardCorners(gray, PATTERN_SIZE)

        if found:
            # Draw all corners
            for i, pt in enumerate(corners):
                x, y = int(pt[0][0]), int(pt[0][1])
                cv2.circle(color, (x, y), 3, (0, 255, 0), -1)
                cv2.putText(color, str(i), (x+5, y-5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,0), 1)

            # Highlight current corner to collect
            cx = int(corners[current_idx][0][0])
            cy = int(corners[current_idx][0][1])
            cv2.circle(color, (cx, cy), 10, (0, 0, 255), 2)
            cv2.putText(color, f"Target idx: {current_idx}",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (0,0,255), 2)

        cv2.imshow("Chessboard", color)
        key = cv2.waitKey(1)

        if key == ord('q'):
            break

        if key == ord('n') and found:
            current_idx = (current_idx + 1) % (PATTERN_SIZE[0] * PATTERN_SIZE[1])
            print(f"[Info] Saved corner idx {current_idx}")

            # --- Get camera 3D coordinates ---
            px, py = corners[current_idx][0]
            px, py = int(px), int(py)
            depth_val = depth.get_distance(px, py)

            cam_pt = rs.rs2_deproject_pixel_to_point(intr, [px, py], depth_val)
            camera_points.append(cam_pt)
            with open(json_file, "a") as f:
                    f.write(json.dumps({f"camera_idx{current_idx}": cam_pt}) + "\n")
                    f.write(json.dumps({f"robot_idx{current_idx}": 0}) + "\n")

            print(f"[OK] Saved paired point: cam={cam_pt}")

    pipeline.stop()
else:
    import json
    import numpy as np

    # -------------------------
    # Read JSONL: camera / robot points
    # -------------------------
    camera_points = {}
    robot_points = {}

    with open(json_file, "r") as f:
        for line in f:
            data = json.loads(line)
            key, value = list(data.items())[0]
            
            idx = int("".join(ch for ch in key if ch.isdigit()))
            
            if "camera" in key:
                camera_points[idx] = value
            else:
                robot_points[idx] = value

    # -------------------------
    # Sort and convert to numpy arrays
    # -------------------------
    camera_xyz = np.array([camera_points[i] for i in sorted(camera_points)])
    robot_xyz  = np.array([robot_points[i]  for i in sorted(robot_points)])

    # -------------------------
    # Solve R / t
    # -------------------------
    camera_center = camera_xyz.mean(axis=0)
    robot_center  = robot_xyz.mean(axis=0)

    camera_shifted = camera_xyz - camera_center
    robot_shifted  = robot_xyz - robot_center

    H = camera_shifted.T @ robot_shifted
    U, S, Vt = np.linalg.svd(H)

    rotation = Vt.T @ U.T

    # Right-hand system correction
    if np.linalg.det(rotation) < 0:
        Vt[2] *= -1
        rotation = Vt.T @ U.T

    translation = robot_center - rotation @ camera_center

    # -------------------------
    # Extrinsic matrix 4×4
    # -------------------------
    extrinsic = np.eye(4)
    extrinsic[:3, :3] = rotation
    extrinsic[:3, 3] = translation

    print("\n=== Camera → Robot Extrinsic Matrix ===")
    print(extrinsic)

    # -------------------------
    # Error evaluation
    # -------------------------
    pred_robot_points = (rotation @ camera_xyz.T).T + translation
    errors = np.linalg.norm(pred_robot_points - robot_xyz, axis=1)


    # try to reconstruct the positions
    from wrs.robot_sim.robots.xarmlite6_wg.x6wg2 import XArmLite6WG2
    import wrs.basis.robot_math as rm
    import wrs.visualization.panda.world as wd
    import wrs.modeling.geometric_model as mgm

    base = wd.World(cam_pos=[2, 0, 1], lookat_pos=[0, 0, 0])
    mgm.gen_frame().attach_to(base)
    robot = XArmLite6WG2(pos=np.array([0, 0, 0]), enable_cc=True)
    rot = np.eye(3)
    for pos in pred_robot_points:
        mgm.gen_frame(pos=pos, rotmat=rot).attach_to(base)
    base.run()
    print("\nMean error:", errors.mean())
    print("Max error:", errors.max())
    print("Min error:", errors.min())

