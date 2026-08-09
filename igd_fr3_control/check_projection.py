import numpy as np
import sys

depth_path = sys.argv[1]
extr_path = sys.argv[2]
fx, fy, cx, cy = 640.5, 640.5, 641.2, 366.9

depth = np.load(depth_path).astype(np.float32)
if depth.max() > 100:
    depth = depth / 1000.0

h, w = depth.shape
# sample a small patch around the image center, ignore zeros
patch = depth[h//2-5:h//2+5, w//2-5:w//2+5]
valid = patch[patch > 0]
if len(valid) == 0:
    print("No valid depth near image center — pick another pixel.")
    sys.exit(1)
z = float(np.median(valid))

u, v = w/2.0, h/2.0
x = (u - cx) * z / fx
y = (v - cy) * z / fy
point_cam = np.array([x, y, z, 1.0])

T = np.load(extr_path).astype(np.float64)  # task -> camera
# invert to go camera -> task
T_inv = np.linalg.inv(T)
point_task = T_inv @ point_cam

print(f"depth at center: {z:.3f} m")
print(f"point in camera frame: {point_cam[:3]}")
print(f"point in task frame:   {point_task[:3]}")
print("Expect roughly: X,Y in [0, 0.30], Z small positive (near table, e.g. 0.0-0.15)")