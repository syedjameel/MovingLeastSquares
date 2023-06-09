#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include "include/MovingLeastSquares.h"

using Matrix = std::vector<std::vector<float>>;
using MatrixPair = std::pair<Matrix, Matrix>;

void MovingLeastSquares::setParameters(float search_radius)
{
    search_radius_ = search_radius;
}

MatrixPair MovingLeastSquares::apply(const Matrix &point_cloud)
{
    // Converting to pcl point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_pcl (new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < point_cloud.size(); ++i)
    {
        pcl::PointXYZ point{point_cloud[i][0], point_cloud[i][1], point_cloud[i][2]};
        point_cloud_pcl->push_back(point);
    }

    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);

    // Reconstruction of the surface of a point cloud by the MovingLeastSquares
    pcl::PointCloud<pcl::PointNormal>::Ptr smoothed_pcloud(new pcl::PointCloud<pcl::PointNormal>);

	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> filter;
	filter.setComputeNormals(true);
	filter.setInputCloud(point_cloud_pcl);
	filter.setPolynomialOrder(3);
	filter.setSearchMethod(kdtree);
	filter.setSearchRadius(search_radius_);

    filter.process(*smoothed_pcloud);

    std::vector<std::vector<float>> point_cloud_vec;
    std::vector<std::vector<float>> normals_vec;
    for (size_t i = 0; i < smoothed_pcloud->points.size(); ++i)
    {

        float p_x = smoothed_pcloud->points[i].x;
        float p_y = smoothed_pcloud->points[i].y;
        float p_z = smoothed_pcloud->points[i].z;
        float n_x = smoothed_pcloud->points[i].normal_x;
        float n_y = smoothed_pcloud->points[i].normal_y;
        float n_z = smoothed_pcloud->points[i].normal_z;
        std::vector<float> point{p_x, p_y, p_z};
        std::vector<float> normal{n_x, n_y, n_z};
        point_cloud_vec.push_back(point);
        normals_vec.push_back(normal);
    }

    return MatrixPair(point_cloud_vec, normals_vec);
}
