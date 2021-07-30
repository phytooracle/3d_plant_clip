#!/usr/bin/env python3
"""
Author : emmanuelgonzalez
Date   : 2021-01-06
Purpose: 3D plant clip
"""

import argparse
import os
import sys
import open3d as o3d
from osgeo import ogr, osr, gdal
import geopandas as gpd
import pandas as pd
import numpy as np
import fiona
import pyproj
from shapely.ops import transform
from shapely import geometry
import utm


# --------------------------------------------------
def get_args():
    """Get command-line arguments"""

    parser = argparse.ArgumentParser(
        description='3D plant clip',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument('pcd',
                        metavar='pcd',
                        help='Merged point cloud')

    parser.add_argument('-c',
                        '--csv',
                        help='CSV containing plant locations (Latitude, Longitude)',
                        metavar='csv',
                        type=str,
                        required=True)

    parser.add_argument('-d',
                        '--date',
                        help='Scan date to find the closest RGB detections.',
                        metavar='date',
                        type=str,
                        required=True)

    parser.add_argument('-o',
                        '--outdir',
                        help='Output directory',
                        metavar='outdir',
                        type=str,
                        default='plantclip_out')

    return parser.parse_args()


# --------------------------------------------------
def nearest_date_df(csv_path, query_date):

    df = pd.read_csv(csv_path)
    df['date'] = pd.to_datetime(df['date'])
    df = df.set_index('date')
    dates = pd.DataFrame(df.index.unique()).set_index('date')
    focal_date = dates.iloc[dates.index.get_loc(pd.to_datetime(query_date), method='nearest')].name.date().strftime("%Y-%m-%d")
    print(f'Closest date is {focal_date}.')
    focal_df = df.loc[focal_date]

    return focal_df


# --------------------------------------------------  
def create_pcd_polygon(pcd):

    multipolygon = ogr.Geometry(ogr.wkbMultiPolygon)

    max_x, max_y, _ = pcd.get_max_bound()
    min_x, min_y, _ = pcd.get_min_bound()

    point_list = [max_x, min_x, max_y, min_y]
    geom = geometry.Polygon([[max_x, max_y], [max_x, min_y], [min_x, min_y], [min_x, max_y]])

    return geom


# --------------------------------------------------
def create_bbox_polygon(min_x, max_y, max_x, min_y, pcd):

    bounding_polygon = np.array([[max_x, max_y, 0],
                                 [max_x, min_y, 0],
                                 [min_x, min_y, 0],
                                 [min_x, max_y, 0]]).astype('float64')
    print(bounding_polygon)
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
def find_intersection(poly, df):
    intersecting_plants = {}
    cnt = 0

    for i, row in df.iterrows():

        lat = row['lat']
        lon = row['lon']
        plot = row['plot']
        plant_id = row['plant_name']
        nw_lat = row['nw_lat']
        nw_lon = row['nw_lon']
        se_lat = row['se_lat']
        se_lon = row['se_lon']

        utm_x, utm_y, _, _ = utm.from_latlon(lat, lon, 12, 'N')
        point = geometry.Point(utm_x, utm_y)

        if point.within(poly):
            cnt += 1

            intersecting_plants[cnt] = {
                'plant_id': plant_id,
                'plot': plot,
                'nw_lat': nw_lat,
                'nw_lon': nw_lon,
                'se_lat': se_lat,
                'se_lon': se_lon
            }

    return intersecting_plants


# --------------------------------------------------
def latlon_to_utm(lat, lon):
    
    utm_e, utm_n, _, _ = utm.from_latlon(lat, lon)

    return utm_e, utm_n


# --------------------------------------------------
def display_inlier_outlier(cloud, ind):

    inlier_cloud = cloud.select_down_sample(ind)


# --------------------------------------------------
def main():
    """Make a jazz noise here"""

    args = get_args()
    basename = os.path.splitext(os.path.basename(args.pcd))[0].split('_')[0]
    pcd = o3d.io.read_point_cloud(args.pcd)
    multipolygon = create_pcd_polygon(pcd)
    plant_locs = nearest_date_df(args.csv, args.date)
    poly = create_pcd_polygon(pcd)
    int_dict = find_intersection(poly, plant_locs)

    if not os.path.isdir(args.outdir):
        os.makedirs(args.outdir)

    for k, v in int_dict.items():
        plant_id = v['plant_id']
        plant_id = '_'.join([plant_id.strip('( )').replace("',", "").replace("'", "").replace(' ', '_'), basename])
        plot = v['plot']
        min_x, max_y = latlon_to_utm(v['nw_lat'], v['nw_lon'])
        max_x, min_y = latlon_to_utm(v['se_lat'], v['se_lon'])
        out_file = os.path.join(args.outdir, plant_id + '.ply')

        plant = create_bbox_polygon(min_x, max_y, max_x, min_y, pcd)
        o3d.io.write_point_cloud(out_file, plant)


# --------------------------------------------------
if __name__ == '__main__':
    main()
