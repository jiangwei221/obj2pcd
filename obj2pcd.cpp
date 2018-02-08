//simple point cloud sampling for obj file
//part of the code from scratchapixel.com, opengl-tutorial.org and learnopengl.com
//using glm, pcl library
//compile command: make
//sample run command: 
// ./obj2pcd ../models/monkey.obj ../output/monkey.pcd 2000

#include <iostream>
#include <vector>
#include "glm/glm.hpp"
#include "glm/vec3.hpp" // vec3
#include "glm/gtx/string_cast.hpp"
#include "modelloader.h"
#include "sampler.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

using namespace glm;
//do not using namespace std overlap with glm; some common functions abiguous, like min/max

int main (int argc, char **argv)
{
    if (argc < 4)
    {
        pcl::console::print_error ("objfile_path pcdfile_path sample_density optional_flip_flag\n", argv[0]);
        return 1;
    }
    //set sample density
    int sample_density = atoi(argv[3]);
    //TO-DO:
    //random seed
    bool flip = false;
    if(argc > 4)
    {
        flip = atoi(argv[4]);
    }
    Sampler sampler = Sampler(argv[1], flip);

    pcl::PointCloud<pcl::PointNormal> out_cloud = sampler.getPointCloud(sample_density);

    pcl::io::savePCDFileASCII(argv[2], out_cloud);
    std::cerr << "saved " << out_cloud.points.size() << " data points to tartget pcd." << std::endl;
    return 0;
}