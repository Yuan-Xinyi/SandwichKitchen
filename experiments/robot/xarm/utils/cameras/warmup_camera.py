from realsense_d405 import RealSenseD405, load_or_create_config, save_config
import cv2
import numpy as np

EXPOSURE_STEP = 500

if __name__ == "__main__":
    cfg = load_or_create_config()

    cameras = [RealSenseD405(cam_cfg) for cam_cfg in cfg["cameras"]]

    while True:
        frames = [cam.get_color_frame() for cam in cameras]
        combined = np.hstack(frames)
        cv2.imshow("D405 Multi-Camera View", combined)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break

        # Increase exposure
        if key == ord('u'):
            for cam_cfg, cam in zip(cfg["cameras"], cameras):
                new_exp = cam_cfg["exposure"] + EXPOSURE_STEP
                cam.set_exposure(new_exp)
            save_config(cfg)

        # Decrease exposure
        if key == ord('d'):
            for cam_cfg, cam in zip(cfg["cameras"], cameras):
                new_exp = max(1, cam_cfg["exposure"] - EXPOSURE_STEP)
                cam.set_exposure(new_exp)
            save_config(cfg)

    for cam in cameras:
        cam.stop()

    cv2.destroyAllWindows()
