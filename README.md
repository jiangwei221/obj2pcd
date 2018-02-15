# obj2pcd
Simple point cloud sampler for obj files.
### Introduction

**obj2pcd** is a simple c++ program to sample a obj mesh to its point cloud representation. 

It depends on GLM and PCL libraries. GML is provided in this repo, but you need to install PCL on your own. For Mac user, you can use `brew install pcl` to get the library.

Some converter on the web directly convert the mesh vertices into a point cloud. It works for fine meshes, but not good for coarse meshes.

In this implementation, we sample the point cloud on the surface of a triangle, not the 3 vertices.

Here is a visualization of the sampling of a cube consists of 12 traingles.

![cube-vis](https://i.imgur.com/hnjpgQ1.png?1)

### Build

`cd build`

`cmake ..`

`make`

### Usage

`./obj2pcd ../models/monkey_v.obj ../output/monkey.pcd -sample_density 2000`

This command will sample the monkey_v.obj file (only vertex) into a pcd file, 2000 is the sample density. You can adjust the density.

There are two optional argvs. One is normal_flag, you need the interpolated normals in PCD or not, default is on. One is flip_flag, you need to flip the normal direction or not, default is off.

### Note

Because I'm using a very simple model loader from [opengl-tutorial](opengl-tutorial.org), it can only handle the obj file with triangles, no quadrilateral support.

If you want to sample an obj mesh exported by Blender, make sure you triangulate it before you export.

### convertall.py

convertall.py is a simple python script to convert all the obj files under one folder into PCD.

```
usage: convertall.py [-h] [--normals NORMALS] [--flip FLIP]
                     models_dir output_dir sample_density

positional arguments:
  models_dir         models dir
  output_dir         output dir
  sample_density     sample density

optional arguments:
  -h, --help         show this help message and exit
  --normals NORMALS  interpolate normals or not
  --flip FLIP        flip the normals or not
```
Sample usage: `python3 convertall.py ./models ./output 200 --normals 1`  where 200 is the sample density.

### TO-DO
