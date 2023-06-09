#include <iostream>
#include <string>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include "MovingLeastSquares.h"

// Default name of the pcd
std::string pcd_file_name = "../sample_pcds/spring1.pcd";

// Default search radius
double search_radius = 0.1;
//std::string pcd_file_path = "../sample_pcds/";

int main(int argc, char **argv){

    std::cout << "Enter the command as follows" << std::endl;
    std::cout << "./movingleastsquares SEARCH_RADIUS PATH_TO_PCD_FILE" << std::endl << std::endl;
    std::cout << "See the sample as follows:" << std::endl;
    std::cout << "./movingleastsquares 10 ../sample_pcds/region_growing_tutorial.pcd" << std::endl  << std::endl;
    if (argc > 1){
        search_radius = std::stod(argv[1]);
        pcd_file_name = argv[2];
        }
    else{
        std::cout << "The file name is not given, using the default " << pcd_file_name << " from path " << pcd_file_name << std::endl;
        pcd_file_name = "../sample_pcds/spring1.pcd";
        std::cout << "The search radius is not given, using the default " << search_radius << std::endl;

    }

    MovingLeastSquares mls;
    mls.setParameters(search_radius);
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_pcl (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile (pcd_file_name, *point_cloud_pcl);
    std::vector<std::vector<float>> point_cloud_vec;
    for (const auto& point : *point_cloud_pcl) {
        std::vector<float> point_vec = {point.x, point.y, point.z};
        point_cloud_vec.push_back(point_vec);
    }
    //Reconstruct
    MatrixPair result = mls.apply(point_cloud_vec);

    // Converting to pcl point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_pcl_smooth (new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < result.first.size(); ++i)
    {
      pcl::PointXYZ point{result.first[i][0], result.first[i][1], result.first[i][2]};
      point_cloud_pcl_smooth->push_back(point);
    }

    std::string filename = pcd_file_name.substr(0, pcd_file_name.find_last_of("."));

    // Save output
    pcl::io::savePCDFile(filename+"-smooth.pcd", *point_cloud_pcl_smooth);

    std::cout << "Saved the smoothened point cloud to " << filename+"-smooth.pcd" << std::endl;

}