# 3D Point Cloud Plant Clip

This Docker container takes a merged point cloud and plant locations JSON file as input. It outputs individual plant point clouds with their respective plant identification numbers.


## Inputs

* Merged point cloud (.PLY)
* Manually geo-corrected plant locations (.JSON)

## Outputs

* Individual plant point clouds (.PLY)

## Arguments

* Required Arguments:
    * **JSON file containing plant locations:** '-j', '--json'
    * **Merged point cloud:** '-p', '--pcd'
    
* Optional Arguments:
    * **Output Directory:** '-o', '--outdir', default = 'plantclip_out'
