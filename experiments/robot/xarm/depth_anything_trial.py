
from depth_anything_3.api import DepthAnything3
import numpy as np
import matplotlib.pyplot as plt

# Initialize and run inference
model = DepthAnything3.from_pretrained("depth-anything/DA3NESTED-GIANT-LARGE").to("cuda")
prediction = model.inference(["img1.png", "img2.png"])
depth_maps = prediction.depth  # shape: (2, H, W)

depth_map = depth_maps[1]

## --------------------------------------------------------
## 方法一：使用 Matplotlib 进行伪彩色可视化 (推荐用于展示)
## --------------------------------------------------------

def visualize_depth_matplotlib(depth_array, title="Depth Map Visualization"):
    # 1. 归一化深度值到 [0, 1] 范围
    # 钳位（可选）：限制最大和最小深度值以提高对比度
    # 例如：移除极端值，这里直接使用 min/max
    depth_min = depth_array.min()
    depth_max = depth_array.max()

    if depth_max - depth_min > 1e-6:
        # 归一化
        normalized_depth = (depth_array - depth_min) / (depth_max - depth_min)
    else:
        normalized_depth = np.zeros_like(depth_array)
    
    plt.figure(figsize=(10, 8))
    # 使用 'viridis' 颜色图，这是一种常用的感知均匀颜色图
    plt.imshow(normalized_depth, cmap='magma') 
    plt.colorbar(label='Normalized Depth (Near to Far)')
    plt.title(title)
    plt.axis('off') # 不显示坐标轴
    plt.show()

# 执行 Matplotlib 可视化
visualize_depth_matplotlib(depth_map, title="DA3 Depth Prediction (Matplotlib)")