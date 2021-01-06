# 3D Plant Clip

This dockerized script takes in a merged point cloud file and a CSV plant locations. It will then output point clouds croped out of the original based on the plant locations given resulting in point clouds of each plant.


## Inputs

* Merged point cloud
* CSV of plant locations

## Outputs

* Individual plant point clouds

## Arguments

* Positional Arguments:
    * **Merged point cloud:** 'pcd'
* Required Arguments:
    * **CSV containing plant locations (Latitude, Longitude):** '-c', '-csv'
* Optional Arguments:
    * **Output Directory:** '-o', '--outdir', default = 'plantclip_out'
