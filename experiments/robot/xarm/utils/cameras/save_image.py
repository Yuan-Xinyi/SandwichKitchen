'''-------------------------------------------------------------------
Save RGB image from multiple RealSense cameras
-------------------------------------------------------------------'''
# import pyrealsense2 as rs
# import numpy as np
# import cv2
# import time

# # -------- initialize cameras --------
# ctx = rs.context()
# devices = ctx.query_devices()
# serials = [d.get_info(rs.camera_info.serial_number) for d in devices]
# print("Detected cameras:", serials)

# pipelines = []
# aligns = []

# for serial in serials:
#     pipe = rs.pipeline()
#     cfg = rs.config()
#     cfg.enable_device(serial)
#     cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
#     cfg.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
#     pipe.start(cfg)

#     pipelines.append(pipe)
#     aligns.append(rs.align(rs.stream.color))

# print("[Camera] All cameras started.")
# time.sleep(2)  # wait for cameras to stabilize

# # -------- read pipeline[1] image --------
# i = 1  # save image from the 2nd pipeline

# frames = pipelines[i].wait_for_frames()
# aligned_frames = aligns[i].process(frames)

# color_frame = aligned_frames.get_color_frame()
# color_img = np.asanyarray(color_frame.get_data())

# # -------- save image --------
# # name = "cam1_rgb_accurate"
# # name = f"cam{1}_rgb_{int(time.time())}"
# name = f"cam{i}_green_and_red"
# cv2.imwrite(f"{name}.png", color_img)
# print(f"Saved {name}.png")


'''-------------------------------------------------------------------
center-to-center pixel distance measurement tool
-------------------------------------------------------------------'''

import cv2
import numpy as np

# save_image.py
# img = cv2.imread("cam1_rgb_accurate.png")
# img = cv2.imread("cam1_rgb_1763959474.png")
img = cv2.imread("cam1_green_and_red.png")
clone = img.copy()

pts = []

def click_event(event, x, y, flags, param):
    global pts, img, clone
    
    if event == cv2.EVENT_LBUTTONDOWN:
        # save point
        pts.append((x, y))
        
        # draw point
        cv2.circle(img, (x, y), 5, (0, 0, 255), -1)
        
        # if two points are selected
        if len(pts) == 2:
            (x1, y1), (x2, y2) = pts
            
            # draw line
            cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # calculate distance
            dist = np.linalg.norm(np.array([x1, y1]) - np.array([x2, y2]))
            print(f"Pixel distance = {dist:.2f}px")
            
            # display distance on image
            mid = ((x1 + x2) // 2, (y1 + y2) // 2)
            cv2.putText(img, f"{dist:.1f}px", mid, 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

        cv2.imshow("Measure", img)

# display image
cv2.imshow("Measure", img)
cv2.setMouseCallback("Measure", click_event)
cv2.waitKey(0)

cv2.destroyAllWindows()

'''-------------------------------------------------------------------
load and measure saved image
-------------------------------------------------------------------'''
# import cv2
# import numpy as np

# # ------------------------------
# # HSV thresholds for red and green
# # ------------------------------

# # Red has two ranges (low + high)
# RED_LOW_1  = np.array([0, 120, 70])
# RED_HIGH_1 = np.array([10, 255, 255])
# RED_LOW_2  = np.array([170, 120, 70])
# RED_HIGH_2 = np.array([180, 255, 255])

# # Green (can be fine-tuned as needed)
# GREEN_LOW  = np.array([35,  80,  40])
# GREEN_HIGH = np.array([85, 255, 255])

# # ------------------------------
# # Calculate centroid from mask
# # ------------------------------
# def get_centroid_from_mask(mask):
#     ys, xs = np.where(mask > 0)
#     if len(xs) == 0:
#         return None
#     return int(np.mean(xs)), int(np.mean(ys))

# # ------------------------------
# # Main function: input image path
# # ------------------------------
# def compute_offset(img_path, color="red"):
#     img = cv2.imread(img_path)
#     if img is None:
#         raise FileNotFoundError(img_path)

#     # fixed target point (pixel coordinates)
#     target_pt = (707, 515)

#     # convert to HSV
#     hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

#     # select mask
#     if color == "red":
#         mask1 = cv2.inRange(hsv, RED_LOW_1, RED_HIGH_1)
#         mask2 = cv2.inRange(hsv, RED_LOW_2, RED_HIGH_2)
#         mask = mask1 | mask2
#     elif color == "green":
#         mask = cv2.inRange(hsv, GREEN_LOW, GREEN_HIGH)
#     else:
#         raise ValueError("color must be 'red' or 'green'")

#     # get centroid from mask
#     c = get_centroid_from_mask(mask)
#     if c is None:
#         print("⚠ No specified color region found")
#         return None

#     # -------------------------
#     # ✨ Compare offset with target point (707, 515)
#     # -------------------------
#     dx = c[0] - target_pt[0]  
#     dy = c[1] - target_pt[1]  

#     print(f"target: {target_pt}")
#     print(f"detection point: {c}")
#     print(f"Pixel offset: dx={dx} px, dy={dy} px")

#     # Visualization
#     vis = img.copy()
#     cv2.circle(vis, target_pt, 6, (255, 255, 255), -1)  # target white point
#     cv2.circle(vis, c, 6, (0, 255, 0), -1)              # detected green point
#     cv2.line(vis, target_pt, c, (0,255,0), 2)

#     cv2.imshow("image", vis)
#     cv2.imshow("mask", mask)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()

#     return dx, dy



# # ------------------------------
# # Example: compute offset
# # ------------------------------
# if __name__ == "__main__":
#     # Read image and detect red region center
#     compute_offset("cam1_rgb_1763959474.png", color="red")
#     # compute_offset("cam1_rgb_accurate.png", color="red")



'''-------------------------------------------------------------------
red need to align with green
-------------------------------------------------------------------'''
# import cv2
# import numpy as np

# # ---- HSV thresholds ----
# RED_LOW_1  = np.array([0, 120, 70])
# RED_HIGH_1 = np.array([10, 255, 255])
# RED_LOW_2  = np.array([170,120,70])
# RED_HIGH_2 = np.array([180,255,255])

# GREEN_LOW  = np.array([40, 50, 50])
# GREEN_HIGH = np.array([85, 255, 255])


# def top_edge(mask):
#     """
#     return the average y position of the top edge in the mask
#     """
#     h, w = mask.shape
#     ys = []

#     for x in range(w):
#         col = mask[:, x]
#         pts = np.where(col > 0)[0]
#         if len(pts) > 0:
#             ys.append(pts[0])   # topmost pixel y in this column

#     if len(ys) == 0:
#         return None

#     return int(np.mean(ys))      # average top edge position


# def visualize_top_edges(img_path):
#     img = cv2.imread(img_path)
#     hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

#     # --- red mask ---
#     mask_r1 = cv2.inRange(hsv, RED_LOW_1, RED_HIGH_1)
#     mask_r2 = cv2.inRange(hsv, RED_LOW_2, RED_HIGH_2)
#     mask_red = mask_r1 | mask_r2

#     # --- green mask ---
#     mask_green = cv2.inRange(hsv, GREEN_LOW, GREEN_HIGH)

#     # --- top edge ---
#     red_top = top_edge(mask_red)
#     green_top = top_edge(mask_green)

#     if red_top is None or green_top is None:
#         print("Missing red or green mask")
#         return

#     # pixel difference (should be equal)
#     dy = green_top - red_top

#     # ---- Visualization ----
#     vis = img.copy()
#     h, w = img.shape[:2]

#     # two horizontal lines
#     cv2.line(vis, (0, red_top), (w, red_top), (0,0,255), 2)
#     cv2.line(vis, (0, green_top), (w, green_top), (0,255,0), 2)

#     # text
#     mid = int((red_top + green_top)/2)
#     cv2.putText(vis, f"{abs(dy):.1f}px", (10, mid),
#                 cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,0,0), 2)

#     cv2.imshow("Top Edge Align", vis)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()

#     print("Red top:", red_top)
#     print("Green top:", green_top)
#     print("dy (green_top - red_top):", dy)

#     return dy

# dy = visualize_top_edges("cam1_green_and_red.png")