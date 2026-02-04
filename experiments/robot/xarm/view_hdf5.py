import h5py
import napari

# Assume the HDF5 file path is correct
file_path = "experiments/robot/xarm/data/stack_r2g_20251124_184050.h5"

with h5py.File(file_path, "r") as f:
    
    # 1. Read color frames (RGB)
    # Dataset name is 'color' based on DataRecorder save method
    # color_frames = f["cam0/color"][:]
    color_frames = f["cam1/color"][:]

    # 2. Read depth frames (uint16)
    # Dataset name is 'depth'
    # depth_frames = f["cam0/depth"][:]
    
    # 3. Read optional robot joint data
    # joints = f["robot/joints"][:]

# ----------------------------------------
# Start napari viewer
# ----------------------------------------
viewer = napari.Viewer()

# Add color images. DataRecorder saved them as RGB.
viewer.add_image(color_frames, name='Main Camera Color', rgb=True)

# To view depth data, uncomment the depth_frames reading line above
# viewer.add_image(depth_frames, name='Main Camera Depth', colormap='turbo', blending='additive')

napari.run()