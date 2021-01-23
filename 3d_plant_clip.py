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
import statistics as stats
from sklearn.cluster import KMeans
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
                        metavar='str',
                        type=str,
                        required=True)

    parser.add_argument('-o',
                        '--outdir',
                        help='Output directory',
                        metavar='str',
                        type=str,
                        default='plantclip_out')

    return parser.parse_args()


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

#---------------------------------------------------
def get_plant_vol(plant_pcd):

    ar_whole_plant = np.asarray(plant_pcd)

    # Find middle z
    z_axis = []

    for i in ar_whole_plant:
        z_axis.append(i[2])

    median_z = stats.median(z_axis)


    # Slicing

    slice_storage_list = []

    a = ar_whole_plant[0][0]
    cnt = 1

    # Create one slice
    for i in ar_whole_plant:
        
        
        if (abs(i[0] - a) < 10) | (len(slice_storage_list) < 1):
            slice_storage_list.append(list(i))
            a = i[0]


        # Cluster the slice, and add clusters that have  a point above the overall
        # z to a 'plant list'
        else:
            # Drop y to flatten here
            flat_slice = copy.deepcopy(slice_storage_list)
            for j in flat_slice:
                del j[1]
            # Cluster here
            km = KMeans(
                n_clusters=5, init='random',
                n_init=10, max_iter=300, 
                tol=1e-04, random_state=0
            )
            
            X = np.array(flat_slice)
            y_km = km.fit_predict(X)

            # Choose cluster
            l0 = []
            l1 = []
            l2 = []
            l3 = []
            l4 = []
            for index, row in enumerate(slice_storage_list):
                # print(index, y_km[index], row)


                if y_km[index] == 0:
                    l0.append(row)
                if y_km[index] == 1:
                    l1.append(row)
                if y_km[index] == 2:
                    l2.append(row)
                if y_km[index] == 3:
                    l3.append(row)
                if y_km[index] == 4:
                    l4.append(row)

            clusters = [l0,l1,l2,l3,l4]

            

            for c in clusters:
                counter = 0
                for i in c:
                    # print(i)
                    # Below worked ok

                    # Tweak these two to get different outputs
                    # best so far is is .01 and 20

                    if i[2] > median_z + (median_z * .01):
                    # if i[2] > median_z:
                        counter += 1
                    
                    if counter >= (len(c)*.2):
                        plant_points.extend(c)
                        break



            slice_storage_list = []

            a = i[0]
            # print(len(plant_points))
            # print(f'Slice {cnt} done')
            # cnt+=1

    pcd_slice = o3d.geometry.PointCloud()
    pcd_slice.points = o3d.utility.Vector3dVector(plant_points)
    pcd_slice_bb = pcd_slice.get_axis_aligned_bounding_box()
    plant_volumes_dict[plant_id] = pcd_slice_bb.volume()


# --------------------------------------------------
def main():
    """Make a jazz noise here"""

    args = get_args()
    pcd = o3d.io.read_point_cloud(args.pcd)
    multipolygon = create_pcd_polygon(pcd)

    plant_locs = pd.read_csv(args.csv)
    print(plant_locs)
    poly = create_pcd_polygon(pcd)
    int_dict = find_intersection(poly, plant_locs)

    if not os.path.isdir(args.outdir):
        os.makedirs(args.outdir)

    plant_volumes_dict = {}

    for k, v in int_dict.items():

        plant_id = v['plant_id']
        plant_id = plant_id.strip('( )').replace("',", "").replace("'", "").replace(' ', '_')
        plot = v['plot']
        min_x, max_y = latlon_to_utm(v['nw_lat'], v['nw_lon'])
        max_x, min_y = latlon_to_utm(v['se_lat'], v['se_lon'])
        out_file = os.path.join(args.outdir, plant_id + '.ply')

        plant = create_bbox_polygon(min_x, max_y, max_x, min_y, pcd)
        o3d.io.write_point_cloud(out_file, plant)

        get_plant_vol(plant)

    plant_volumes_df = pd.DataFrame.from_dict(plant_volumes_dict, orient = 'index', columns = ['plant_vol'])

    plant_volumes_df.to_csv(os.path.basename(args.pcd) + '_plant_vol.csv')






# --------------------------------------------------
if __name__ == '__main__':
    main()
