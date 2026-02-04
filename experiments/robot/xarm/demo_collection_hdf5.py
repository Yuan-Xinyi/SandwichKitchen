import pyrealsense2 as rs
import cv2
import numpy as np

############################################
# Step 1: Find all RealSense devices
############################################
ctx = rs.context()
devices = ctx.query_devices()
serials = [d.get_info(rs.camera_info.serial_number) for d in devices]

print("Found camera serials", serials)
if len(serials) != 2:
    raise RuntimeError("Two RealSense D405 cameras are required, but found {}".format(len(serials)))

############################################
# Step 2: Create a pipeline for each camera
############################################
pipelines = []
configs = []

for serial in serials[:2]:      # Only use the first two
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(serial)

    # width=640, height=480, fps=30
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    pipelines.append(pipeline)
    configs.append(config)

############################################
# Step 3: Start pipelines
############################################
for p, c in zip(pipelines, configs):
    p.start(c)

print("Two D405 cameras have started")

############################################
# Step 4: Loop to read and display frames
############################################
while True:
    frames_rgb = []
    
    for pipeline in pipelines:
        frames = pipeline.wait_for_frames()

        # 提取 RGB 图像
        color_frame = frames.get_color_frame()
        if not color_frame:
            frames_rgb.append(np.zeros((480, 640, 3), dtype=np.uint8))
        else:
            color_image = np.asanyarray(color_frame.get_data())
            frames_rgb.append(color_image)

    # concatenate images horizontally
    combined = np.hstack(frames_rgb)

    cv2.imshow("D405 RGB Streams (Two Cameras)", combined)

    # Press q to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

############################################
# Step 5: Release resources
############################################
for p in pipelines:
    p.stop()

cv2.destroyAllWindows()
