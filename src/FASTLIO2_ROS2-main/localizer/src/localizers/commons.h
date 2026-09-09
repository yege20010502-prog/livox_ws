#pragma once
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// Localization only needs geometry. Using PointXYZ also allows maps exported
// without an intensity field to load without PCL field-mismatch warnings.
using PointType = pcl::PointXYZ;
using CloudType = pcl::PointCloud<PointType>;
using PointVec = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

using M3D = Eigen::Matrix3d;
using V3D = Eigen::Vector3d;
using M3F = Eigen::Matrix3f;
using V3F = Eigen::Vector3f;
using M4F = Eigen::Matrix4f;
using V4F = Eigen::Vector4f;
