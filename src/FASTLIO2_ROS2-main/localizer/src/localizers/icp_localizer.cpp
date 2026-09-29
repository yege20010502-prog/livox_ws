#include "icp_localizer.h"
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <cmath>

ICPLocalizer::ICPLocalizer(const ICPConfig &config) : m_config(config)
{
    m_refine_inp.reset(new CloudType);
    m_refine_tgt.reset(new CloudType);
    m_rough_inp.reset(new CloudType);
    m_rough_tgt.reset(new CloudType);

    // Estimate map -> odom for a ground robot. Unconstrained 6-DoF ICP can
    // incorrectly tilt a scan onto the floor or ceiling.
    using PlanarEstimation =
        pcl::registration::TransformationEstimation2D<PointType, PointType, float>;
    m_rough_icp.setTransformationEstimation(
        std::make_shared<PlanarEstimation>());
    m_refine_icp.setTransformationEstimation(
        std::make_shared<PlanarEstimation>());
    m_rough_icp.setMaxCorrespondenceDistance(
        m_config.rough_max_correspondence_distance);
    m_refine_icp.setMaxCorrespondenceDistance(
        m_config.refine_max_correspondence_distance);
}

CloudType::Ptr ICPLocalizer::removeGroundPlane(const CloudType::Ptr &cloud) const
{
    if (!m_config.remove_ground_plane || cloud->size() <
            static_cast<std::size_t>(m_config.ground_plane_min_points))
        return cloud;

    pcl::SACSegmentation<PointType> segmentation;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setAxis(Eigen::Vector3f::UnitZ());
    segmentation.setEpsAngle(static_cast<float>(15.0 * M_PI / 180.0));
    segmentation.setDistanceThreshold(m_config.ground_plane_distance);
    segmentation.setMaxIterations(100);
    segmentation.setInputCloud(cloud);
    segmentation.segment(*inliers, *coefficients);

    if (inliers->indices.size() <
        static_cast<std::size_t>(m_config.ground_plane_min_points))
        return cloud;

    pcl::ExtractIndices<PointType> extract;
    CloudType::Ptr without_ground(new CloudType);
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*without_ground);
    return without_ground;
}

bool ICPLocalizer::loadMap(const std::string &path)
{
    if (!std::filesystem::exists(path))
    {
        std::cerr << "Map file not found: " << path << std::endl;
        return false;
    }
    pcl::PCDReader reader;
    CloudType::Ptr cloud(new CloudType);
    reader.read(path, *cloud);
    // Use the same obstacle-height band for the target as for live scans.
    // This permits loading map_indoor_clean.pcd while ignoring its floor and
    // ceiling points during ground-robot localization.
    CloudType::Ptr filtered_cloud(new CloudType);
    m_height_filter.setInputCloud(cloud);
    m_height_filter.setFilterFieldName("z");
    m_height_filter.setFilterLimits(-0.33, 0.39);
    m_height_filter.filter(*filtered_cloud);
    cloud = removeGroundPlane(filtered_cloud);
    if (m_config.refine_map_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.refine_map_resolution, m_config.refine_map_resolution, m_config.refine_map_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_refine_tgt);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *m_refine_tgt);
    }

    if (m_config.rough_map_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.rough_map_resolution, m_config.rough_map_resolution, m_config.rough_map_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_rough_tgt);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *m_rough_tgt);
    }
    return true;
}
void ICPLocalizer::setInput(const CloudType::Ptr &cloud)
{
    // Match the live scan to the obstacle-only localization map. The saved
    // map_obstacles.pcd uses this same z band, so exclude floor and ceiling
    // points before correspondence search.
    CloudType::Ptr filtered_cloud(new CloudType);
    m_height_filter.setInputCloud(cloud);
    m_height_filter.setFilterFieldName("z");
    m_height_filter.setFilterLimits(-0.33, 0.39);
    m_height_filter.filter(*filtered_cloud);
    filtered_cloud = removeGroundPlane(filtered_cloud);

    if (m_config.refine_scan_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.refine_scan_resolution, m_config.refine_scan_resolution, m_config.refine_scan_resolution);
        m_voxel_filter.setInputCloud(filtered_cloud);
        m_voxel_filter.filter(*m_refine_inp);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *m_refine_inp);
    }

    if (m_config.rough_scan_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.rough_scan_resolution, m_config.rough_scan_resolution, m_config.rough_scan_resolution);
        m_voxel_filter.setInputCloud(filtered_cloud);
        m_voxel_filter.filter(*m_rough_inp);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *m_rough_inp);
    }
}

bool ICPLocalizer::align(M4F &guess)
{
    m_last_rough_score = std::numeric_limits<double>::infinity();
    m_last_refine_score = std::numeric_limits<double>::infinity();
    m_last_rough_converged = false;
    m_last_refine_converged = false;
    m_last_overlap_ratio = 0.0;
    m_last_overlap_points = 0;
    m_last_input_points = static_cast<int>(m_refine_inp->size());
    CloudType::Ptr aligned_cloud(new CloudType);
    if (m_refine_tgt->empty() || m_rough_tgt->empty() ||
        m_refine_inp->empty() || m_rough_inp->empty())
        return false;
    m_rough_icp.setMaximumIterations(m_config.rough_max_iteration);
    m_rough_icp.setInputSource(m_rough_inp);
    m_rough_icp.setInputTarget(m_rough_tgt);
    m_rough_icp.align(*aligned_cloud, guess);
    m_last_rough_converged = m_rough_icp.hasConverged();
    m_last_rough_score = m_rough_icp.getFitnessScore();
    if (!m_last_rough_converged || m_last_rough_score > m_config.rough_score_thresh)
        return false;
    m_refine_icp.setMaximumIterations(m_config.refine_max_iteration);
    m_refine_icp.setInputSource(m_refine_inp);
    m_refine_icp.setInputTarget(m_refine_tgt);
    m_refine_icp.align(*aligned_cloud, m_rough_icp.getFinalTransformation());
    m_last_refine_converged = m_refine_icp.hasConverged();
    m_last_refine_score = m_refine_icp.getFitnessScore();
    if (!m_last_refine_converged || m_last_refine_score > m_config.refine_score_thresh)
        return false;

    // PCL's fitness score averages only accepted correspondences. A small
    // repeated structure (or the floor) can therefore look excellent even
    // when most of the live scan is outside the saved map. Require a minimum
    // number and fraction of source points to be supported by the map.
    CloudType::Ptr transformed(new CloudType);
    pcl::transformPointCloud(
        *m_refine_inp, *transformed, m_refine_icp.getFinalTransformation());
    pcl::KdTreeFLANN<PointType> map_tree;
    map_tree.setInputCloud(m_refine_tgt);
    const float max_distance_sq = static_cast<float>(
        m_config.refine_max_correspondence_distance *
        m_config.refine_max_correspondence_distance);
    std::vector<int> indices(1);
    std::vector<float> squared_distances(1);
    for (const auto &point : transformed->points)
    {
        if (!pcl::isFinite(point))
            continue;
        if (map_tree.nearestKSearch(point, 1, indices, squared_distances) > 0 &&
            squared_distances[0] <= max_distance_sq)
            ++m_last_overlap_points;
    }
    m_last_overlap_ratio = transformed->empty()
        ? 0.0
        : static_cast<double>(m_last_overlap_points) /
              static_cast<double>(transformed->size());
    if (m_last_overlap_points < m_config.min_overlap_points ||
        m_last_overlap_ratio < m_config.min_overlap_ratio)
        return false;

    guess = m_refine_icp.getFinalTransformation();
    return true;
}
