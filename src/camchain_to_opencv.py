#!/usr/bin/python

import cv2
import numpy as np
import yaml
from os.path import join
from collections import OrderedDict

def load_cam(cam_dict):
    intrinsics = cam_dict["intrinsics"]
    K = np.eye(3)
    np.fill_diagonal(K[:2,:2], intrinsics[:2])
    K[:2,2] = intrinsics[2:]
    D = np.zeros(5)
    D[:4] = cam_dict["distortion_coeffs"]
    return K, D

def dump_matrix(mat):
    ret = OrderedDict()
    dim = mat.shape
    ret["rows"] = dim[0]
    ret["cols"] = dim[1] if len(dim) > 1 else 1
    ret["data"] = mat.flatten().tolist()
    return ret

def dump_cam(K, D, R, P, Size, name):
    ret = OrderedDict()
    ret["image_width"], ret["image_height"] = Size
    ret["camera_name"] = name
    ret["camera_matrix"] = dump_matrix(K)
    ret["distortion_model"] = "plumb_bob"
    ret["distortion_coefficients"] = dump_matrix(D)
    ret["rectification_matrix"] = dump_matrix(R)
    ret["projection_matrix"] = dump_matrix(P)
    return ret

def convert(calib, alpha):
    K0, D0 = load_cam(calib["cam0"])
    K1, D1 = load_cam(calib["cam1"])
    T_01 = np.linalg.inv(np.array(calib["cam1"]["T_cn_cnm1"]))
    R = T_01[:3,:3]
    T = T_01[:3,3]
    Size = tuple(calib["cam0"]["resolution"])  # Assumes both cameras have same resolution

    R0, R1, P0, P1 = cv2.stereoRectify(cameraMatrix1=K0, cameraMatrix2=K1,  distCoeffs1=D0, distCoeffs2=D1, imageSize=Size, R=R, T=T, flags=cv2.CALIB_ZERO_DISPARITY, alpha=alpha)[:4]

    return dump_cam(K0, D0, R0, P0, Size, "cam_fl"), dump_cam(K1, D1, R1, P1, Size, "cam_fr")

class BracedOrderedDict(OrderedDict):
    pass
yaml.add_representer(BracedOrderedDict, lambda self, d: "{" + self.represent_dict(d.iteritems()) + "}")
def okvis_camera(calib):
    ret = OrderedDict()
    ret["T_SC"] = np.linalg.inv(calib["T_cam_imu"]).flatten().tolist()
    ret["image_dimension"] = calib["resolution"]
    # Excluding distortion coefficients because cameras are already undistorted
    ret["focal_length"] = calib["intrinsics"][:2]
    ret["principal_point"] = calib["intrinsics"][2:]
    return ret

def okvis_partial(calib):
    ret = OrderedDict()
    cam_fl = okvis_camera(calib["cam0"])
    cam_fr = okvis_camera(calib["cam1"])
    ret["cameras"] = [cam_fl, cam_fr]
    ret["image_delay"] = max(calib["cam%d" % n]["timeshift_cam_imu"] for n in (0,1))
    return ret

def main():
    import argparse
    parser = argparse.ArgumentParser(description="Convert kalibr camera camchain file to format readable by OpenCV in ROS camera_calibration_parsers")
    parser.add_argument("infile", metavar="in", help="Input kalibr camchain yaml file")
    parser.add_argument("outdir", metavar="outdir", default=".", nargs="?", help="Output directory OpenCV yaml file")
    parser.add_argument("--alpha", type=float, default=1.0, help="Stereo rectify alpha")
    parser.add_argument("--stereo_only", action="store_true", help="Only generate opencv files from stereo")
    args = parser.parse_args()

    yaml.add_representer(OrderedDict, lambda self, d: self.represent_dict(d.iteritems()))

    in_calib = yaml.load(file(args.infile, 'r'))
    out_calib_fl, out_calib_fr = convert(in_calib, alpha=args.alpha)
    yaml.dump(out_calib_fl, file(join(args.outdir, "cam_fl.yaml"), 'w+'))
    yaml.dump(out_calib_fr, file(join(args.outdir, "cam_fr.yaml"), 'w+'))

    if not args.stereo_only:
        out_calib_okvis_partial = okvis_partial(in_calib)
        yaml.dump(out_calib_okvis_partial, file(join(args.outdir, "config_okvis.yaml"), 'w+'))

if __name__=='__main__':
    main()