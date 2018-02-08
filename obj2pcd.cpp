//simple point cloud sampling for obj file
//part of the code from scratchapixel.com, opengl-tutorial.org and learnopengl.com
//using glm, pcl library
//compile command: make
//sample run command: 
// ./obj2pcd ../models/part.obj 2000 0
// ./pcd_viewer ../output/part.pcd -ps 5 -normals 1 -normals_scale 10
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
    if (argc < 3)
    {
        pcl::console::print_error ("pcdfile sample_density optional_flip_flag\n", argv[0]);
        return 1;
    }
    //how many samples
    int sample_density = atoi(argv[2]);
    //TO-DO:
    //random seed
    bool flip = false;
    if(argc > 3)
    {
        flip = atoi(argv[3]);;
    }
    Sampler sampler = Sampler(argv[1], flip);

    pcl::PointCloud<pcl::PointNormal> testcloud = sampler.getPointCloud(sample_density);

    pcl::io::savePCDFileASCII("../output/test_pcd.pcd", testcloud);
    std::cerr << "Saved " << testcloud.points.size() << " data points to test_pcd.pcd." << std::endl;
    return 0;
}