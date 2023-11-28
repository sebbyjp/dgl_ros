import sys
sys.path.append('/simply_ws/src/cgn_pytorch')
from cgn_pytorch import from_pretrained, inference
import numpy as np

cgn, _, _ = from_pretrained()
arrays = np.load('/simply_ws/src/cgn_pytorch/cgn_pytorch/scenes/005274.npz', allow_pickle=True)
[k0, k1, k2, k3, k4, k5] = arrays.files
pcd_list, _, obj_mask, mean, cam_pose, _ = arrays[k0], arrays[k1], arrays[k2], arrays[k3], arrays[k4], arrays[k5]
pointcloud = pcd_list[0]
print(pointcloud.shape)
grasps, confidence = inference(cgn, pointcloud)
print(grasps)
