#pragma once
#include <pcl/surface/mls.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <vector>
#include <utility>
#include <iostream>

using Matrix = std::vector<std::vector<float>>;
using MatrixPair = std::pair<Matrix, Matrix>;

class MovingLeastSquares
{
public:
    void setParameters(float search_radius);
    MatrixPair apply(const Matrix &point_cloud);
private:
   float search_radius_;
};