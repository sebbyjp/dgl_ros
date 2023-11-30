import sys
sys.path.append('/simply_ws/src/cgn_pytorch')
from cgn_pytorch import from_pretrained, inference  # noqa: E402
import numpy as np  # noqa: E402
from pprint import pprint # noqa: E402
cgn, _, _ = from_pretrained()
arrays = np.load('/simply_ws/src/cgn_pytorch/cgn_pytorch/scenes/005274.npz', allow_pickle=True)
[k0, k1, k2, k3, k4, k5] = arrays.files
pcd_list, _, obj_mask, mean, cam_pose, _ = arrays[k0], arrays[k1], arrays[k2], arrays[k3], arrays[k4], arrays[k5]
pprint(vars(arrays))
pointcloud = pcd_list[0]
print(pointcloud.shape)
grasps, confidence, downsample = inference(cgn, pointcloud, threshold=0.5,visualize=True)
print(grasps.shape)
print("Found {} grasps".format(grasps.shape[0]))
