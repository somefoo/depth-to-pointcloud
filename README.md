# depth-to-pointcloud
![blender-to-point-cloud](https://user-images.githubusercontent.com/50917034/130279207-4d3ee733-a2da-4a83-9750-9b38b9a99cc0.jpg)
*The image above shows a scene rendered in [Blender](https://www.blender.org/)*, stored as an OpenEXR file with a Z Buffer and converted into a point cloud with some noise.

A simple tool to convert a linear depth image (z-depth) to a point cloud (using the OpenEXR image format)

# Requirements
* GCC with basic C++20 support (9.3 from the Ubuntu 20.04 repository works)
* CMake 3.14 or greater
* That is pretty much it, all other dependencies will be automatically downloaded and compiled by CMake/Hunter.

# How to use
## Compile
``` bash
git clone https://github.com/somefoo/depth-to-pointcloud.git
cd depth-to-pointcloud
mkdir build
cd build
cmake ..
make
```
## Application usage
```
depth-to-pointcloud - OpenEXR with Z Buffer to PCD point cloud converter.

Usage: run-depth-to-pointcloud --input [FILE1] --output [FILE2]
Reads FILE1 to generate a .pcd file FILE2

Options: 
  --focal-length <float>[=<50>]     Pinhole-camera focal length
  --sensor-width  <float>[=<36>]    Pinhole-camera sensor size
  --upper-cut <float>[=<infinity>]  Cuts off points too far away
  --lower-cut <float>[=<-infinity>] Cuts off points too close
  --keep-fraction <float>[=<1.0>]   Percentage of points used
                                     has to be in [0,1]
  --add-noise <float>[=<0.0>]       Adds gaussian noise
                                     has to be in [0,infinity]
  --rgb <float>[=<4.2108e+06>]      Sets color of the points

 -h, --help                         Prints this message


Example 1:
./run-depth-to-pointcloud --input image.exr --output pointcloud.pcd

Example 2 (keep 50% of points, add noise with variance of 2.0):
./run-depth-to-pointcloud --input image.exr --output pointcloud.pcd \
  --sensor-width 10 --focal-length 42 --keep-fraction 0.5 --add-noise 2.0 \
  --lower-cut 100 --upper-cut 65500 

Example 3 (output will default to image.pcd):
./run-depth-to-pointcloud --input image.exr

```

