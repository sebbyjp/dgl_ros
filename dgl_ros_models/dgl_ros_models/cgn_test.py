import sys

sys.path.append("/simply_ws/src/cgn_pytorch")
from cgn_pytorch import from_pretrained, inference  # noqa: E402
import numpy as np  # noqa: E402
from pprint import pprint  # noqa: E402
import argparse  # noqa: E402

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--checkpoint_path",
        type=str,
        default="/simply_ws/src/cgn_pytorch/cgn_pytorch/checkpoints/current.pth",
        help="path to load model from",
    )
    parser.add_argument('--pcd_path', type=str, default='/simply_ws/src/cgn_pytorch/cgn_pytorch/scenes/005274.npz',help='path to load pcd from')
    parser.add_argument(
        "--threshold", type=float, default=0.5, help="threshold for grasp confidence"
    )
    parser.add_argument(
        "--gripper_depth", type=float, default=0.035, help="gripper depth"
    )
    parser.add_argument(
        "--gripper_width", type=float, default=0.035, help="gripper width"
    )
    parser.add_argument(
        "--visualize",
        action="store_true",
        default=False,
        help="whether or not to visualize in meshcat",
    )
    parser.add_argument(
        "--max_grasps", type=int, default=0, help="maximum number of grasps to return"
    )
    import numpy as np
    points =  np.load('/simply_ws/src/cgn_pytorch/cgn_pytorch/gripper/control_points/panda.npy', encoding='bytes')
    print(points.shape)
    print(points)

    args = parser.parse_args()
    cgn, _, _ = from_pretrained(args.checkpoint_path, device="cpu")

    arrays = np.load(args.pcd_path, allow_pickle=True)
    [k0, k1, k2, k3, k4, k5] = arrays.files
    pcd_list, _, obj_mask, mean, cam_pose, _ = (
        arrays[k0],
        arrays[k1],
        arrays[k2],
        arrays[k3],
        arrays[k4],
        arrays[k5],
    )
    pprint(vars(arrays))
    pointcloud = pcd_list[0]
    print(pointcloud.shape)
    grasps, confidence, downsample = inference(
        cgn,
        pointcloud,
        threshold=args.threshold,
        visualize=args.visualize,
        gripper_depth=args.gripper_depth,
        gripper_width=args.gripper_width,
    )
    print(grasps.shape)
    print("Found {} grasps".format(grasps.shape[0]))
    print("Confidence: {}".format(confidence))
