#include "icp_localizer.h"

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
    cloud = filtered_cloud;
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
    CloudType::Ptr aligned_cloud(new CloudType);
    if (m_refine_tgt->size() == 0 || m_rough_tgt->size() == 0)
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
    guess = m_refine_icp.getFinalTransformation();
    return true;
}
