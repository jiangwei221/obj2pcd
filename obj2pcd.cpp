//simple point cloud sampler for obj file
//wei jiang
//part of the code from scratchapixel.com, opengl-tutorial.org and learnopengl.com
//using glm, pcl library
//compile command: make
//sample run command:
// default sample density is 100
// ./obj2pcd ../models/monkey_v.obj ../output/monkey.pcd

// use sample density as 200
// ./obj2pcd ../models/monkey_v.obj ../output/monkey.pcd -sample_density 200

// no normals interpolation
// ./obj2pcd ../models/monkey_v.obj ../output/monkey.pcd -sample_density 200 -normal_flag 0

// pcd_viewer
// ./pcd_viewer ../output/monkey.pcd -ps 5 -normals 1 -normals_scale 1

#include <iostream>
#include <vector>
#include <stdlib.h> /* srand, rand */
#include <time.h>   /* time */
#include "glm/glm.hpp"
#include "glm/gtx/string_cast.hpp"
#include "modelloader.h"
#include "sampler.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>

using namespace glm;
//do not using namespace std overlap with glm; some common functions abiguous, like min/max

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        pcl::console::print_error("argv: objfile_path pcdfile_path sample_density optioanl_normal_flag optional_flip_flag\n", argv[0]);
        return 1;
    }

    //set sample density
    int sample_density = 100;
    pcl::console::parse_argument(argc, argv, "-sample_density", sample_density);

    bool normal_flag = true;
    pcl::console::parse_argument(argc, argv, "-normal_flag", normal_flag);

    bool flip = false;
    pcl::console::parse_argument(argc, argv, "-flip_flag", flip);

    std::cout << "sample_density: " << sample_density << std::endl;
    std::cout << "normal_flag: " << normal_flag << std::endl;
    std::cout << "flip_flag: " << flip << std::endl;

    //set random seed
    srand(time(NULL));

    Sampler sampler = Sampler(argv[1], normal_flag, flip);

    pcl::PointCloud<pcl::PointNormal> out_cloud = sampler.getPointCloud(sample_density);

    pcl::io::savePCDFileASCII(argv[2], out_cloud);
    std::cerr << "saved " << out_cloud.points.size() << " data points to " << argv[2] << "." << std::endl;
    return 0;
}
