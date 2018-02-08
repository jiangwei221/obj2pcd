# obj2pcd
Simple point cloud sampler for obj files.
### Introduction

**obj2pcd** is a simple c++ program to sample a obj mesh(with vertex and vertex normal) to its point cloud representation. 

It depends on GLM and PCL libraries. GML is provided in this repo, but you need to install PCL on your won. For Mac user, you can use `brew install pcl` to get the library.

Some converter on the web directly convert the mesh vertex into a point cloud. It works for fine meshes, but not good for coarse meshes.

In this implementation, we sample the point cloud on the surface of a triangle, not the 3 vertices.

### Build

`cd build`

`cmake ..`

`make`

### Usage

`./obj2pcd ../models/monkey.obj ../output/monkey.pcd 2000`

This command will sample the monkey.obj file into a pcd file, 2000 is the sample density. You can adjust the density.

### Note

Because I'm using a very simple model loader from [opengl-tutorial](opengl-tutorial.org), it can only handle the obj file contains vertex and normal data. If your obj have no normal data, it will fail. If your obj file have texture coord, it will fail. 

Also, replace the `//` to `/` in the obj file.

### TO-DO

A more robust model loader.

Random seed.