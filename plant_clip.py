#!/usr/bin/env python3
"""
Author : Emmanuel Gonzalez
Date   : 2021-07-30
Purpose: 3D Individual plant clipping
"""

import argparse
import os
import sys
import pandas as pd
import open3d as o3d
import numpy as np
import glob
import json

# --------------------------------------------------
def get_args():
    """Get command-line arguments"""

    parser = argparse.ArgumentParser(
        description='3D individual plant clip',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument('-p',
                        '--pcd',
                        help='Full resolution individual pass point cloud.',
                        metavar='str',
                        type=str,
                        required=True)

    parser.add_argument('-j',
                        '--json',
                        help='JSON file containing plant locations.',
                        metavar='str',
                        type=str,
                        required=True)

    parser.add_argument('-o',
                        '--out_dir',
                        help='Output directory.',
                        metavar='str',
                        type=str,
                        default='plant_clip_out')



    return parser.parse_args()


# --------------------------------------------------
def create_bbox_polygon(UL, UR, LL, LR, pcd):

    bounding_polygon = np.array([[UL[0], UL[1], 0],
                                 [UR[0], UR[1], 0],
                                 [LR[0], LR[1], 0],
                                 [LL[0], LL[1], 0]]).astype('float64')

    _, _, max_z = pcd.get_max_bound()
    _, _, min_z = pcd.get_min_bound()
    vol = o3d.visualization.SelectionPolygonVolume()
    vol.orthogonal_axis = "Z"
    vol.axis_max = max_z
    vol.axis_min = min_z
    vol.bounding_polygon = o3d.utility.Vector3dVector(bounding_polygon)

    plant = vol.crop_point_cloud(pcd)

    return plant


# --------------------------------------------------
def main():
    """Make a jazz noise here"""

    args = get_args()

    if not os.path.isdir(args.out_dir):
        os.makedirs(args.out_dir)

    super_dict = {}
    
    basename = os.path.splitext(args.json.replace('_plant_locations', ''))
    pcd_path = os.path.join(args.pcd)

    with open(args.json) as f:
        data = json.load(f)

        for k, v in data.items():
            plant_id = k
            pcd = o3d.io.read_point_cloud(pcd_path)
            
            plant_pcd = create_bbox_polygon(v['UL'], v['UR'], v['LL'], v['LR'], pcd)

            out_path = os.path.join(args.out_dir, ''.join([plant_id, '.ply']))
            o3d.io.write_point_cloud(out_path, plant_pcd)


# --------------------------------------------------
if __name__ == '__main__':
    main()
