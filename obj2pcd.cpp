//simple point cloud sampler for obj file
//wei jiang
//part of the code from scratchapixel.com, opengl-tutorial.org and learnopengl.com
//using glm, pcl library
//compile command: make
//sample run command:
// ./obj2pcd ../models/monkey_v.obj ../output/monkey.pcd 2000
// ./pcd_viewer ../output/monkey.pcd -ps 5 -normals 1 -normals_scale 1

#include <iostream>
#include <vector>
#include <stdlib.h> /* srand, rand */
#include <time.h>   /* time */
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

int main(int argc, char **argv)
{
    if (argc < 4)
    {
        pcl::console::print_error("argv: objfile_path pcdfile_path sample_density optioanl_normal_flag optional_flip_flag\n", argv[0]);
        return 1;
    }
    //set sample density
    int sample_density = atoi(argv[3]);

    //set random seed
    srand(time(NULL));

    //set two optional flags
    bool normal_flag = true;
    if (argc > 4)
    {
        normal_flag = atoi(argv[4]);
    }

    bool flip = false;
    if (argc > 5)
    {
        flip = atoi(argv[5]);
    }

    Sampler sampler = Sampler(argv[1], normal_flag, flip);

    pcl::PointCloud<pcl::PointNormal> out_cloud = sampler.getPointCloud(sample_density);

    pcl::io::savePCDFileASCII(argv[2], out_cloud);
    std::cerr << "saved " << out_cloud.points.size() << " data points to tartget pcd." << std::endl;
    return 0;
}