#pragma once
#include "commons.h"
#include <filesystem>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <limits>

struct ICPConfig
{
    double refine_scan_resolution = 0.1;
    double refine_map_resolution = 0.1;
    double refine_score_thresh = 0.1;
    int refine_max_iteration = 10;

    double rough_scan_resolution = 0.25;
    double rough_map_resolution = 0.25;
    double rough_score_thresh = 0.2;
    int rough_max_iteration = 5;
};

class ICPLocalizer
{
public:
    ICPLocalizer(const ICPConfig &config);
    
    bool loadMap(const std::string &path);
    
    void setInput(const CloudType::Ptr &cloud);

    bool align(M4F &guess);
    ICPConfig &config() { return m_config; }
    CloudType::Ptr roughMap() { return m_rough_tgt; }
    CloudType::Ptr refineMap() { return m_refine_tgt; }
    double roughScore() const { return m_last_rough_score; }
    double refineScore() const { return m_last_refine_score; }
    bool roughConverged() const { return m_last_rough_converged; }
    bool refineConverged() const { return m_last_refine_converged; }


private:
    ICPConfig m_config;
    pcl::VoxelGrid<PointType> m_voxel_filter;
    pcl::PassThrough<PointType> m_height_filter;
    pcl::IterativeClosestPoint<PointType, PointType> m_refine_icp;
    pcl::IterativeClosestPoint<PointType, PointType> m_rough_icp;
    CloudType::Ptr m_refine_inp;
    CloudType::Ptr m_rough_inp;
    CloudType::Ptr m_refine_tgt;
    CloudType::Ptr m_rough_tgt;
    std::string m_pcd_path;
    double m_last_rough_score{std::numeric_limits<double>::infinity()};
    double m_last_refine_score{std::numeric_limits<double>::infinity()};
    bool m_last_rough_converged{false};
    bool m_last_refine_converged{false};
};
