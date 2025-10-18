#include "rs_viewer/rs_viewer.h"
#include <pcl/console/print.h>
#include <cmath>
#include <algorithm>
#include <Eigen/StdVector>
#include <rs_driver/common/rs_log.hpp>

namespace robosense::viewer {

LidarViewer::LidarViewer(const std::string& window_name)
    : window_name_(window_name),
      state_(ViewerState::UNINITIALIZED),
      cloud_callback_(nullptr) {
    // 禁用PCL/VTK日志输出，避免干扰主程序
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
}

LidarViewer::~LidarViewer() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (viewer_) {
        viewer_->removeAllPointClouds();
        viewer_->removeAllCoordinateSystems();
        viewer_->close();
        viewer_.reset();
    }
    pcl_cloud_.reset();
    color_handler_.reset();
    state_ = ViewerState::UNINITIALIZED;
}

bool LidarViewer::init() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != ViewerState::UNINITIALIZED) {
        return false;
    }

    // 创建可视化窗口
    viewer_ = std::make_shared<pcl::visualization::PCLVisualizer>(window_name_);
    if (!viewer_ || viewer_->wasStopped()) {
        return false;
    }

    // 设置背景色与坐标系
    viewer_->setBackgroundColor(0.0, 0.0, 0.0);
    viewer_->addCoordinateSystem(1.0);
    viewer_->initCameraParameters();
    viewer_->setCameraPosition(-10.0, 0.0, 2.0,   // 相机位置
                               0.0, 0.0, 0.0,     // 观察目标
                               0.0, 0.0, 1.0);    // 上方向

    // 初始化点云容器
    pcl_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl_cloud_->height = 1;
    pcl_cloud_->width = 0;
    pcl_cloud_->is_dense = true;

    // 创建强度着色器并添加点云到视图
    color_handler_ = std::make_shared<pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>>(
        pcl_cloud_, "intensity");
    viewer_->addPointCloud<pcl::PointXYZI>(pcl_cloud_, *color_handler_, "rs_cloud");
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                             2, "rs_cloud");

    state_ = ViewerState::INITIALIZED;
    return true;
}

bool LidarViewer::start() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != ViewerState::INITIALIZED) {
        return false;
    }
    state_ = ViewerState::RUNNING;
    return true;
}

void LidarViewer::processAndShowPointCloud(PointCloudMsgPtr curr_msg) {
#ifdef RS_TIME_RECORD
    auto start_time = std::chrono::high_resolution_clock::now();
#endif

    // 状态或数据无效则跳过
    if (state_ != ViewerState::INITIALIZED && state_ != ViewerState::RUNNING) {
        return;
    }
    if (!curr_msg || curr_msg->points.empty()) {
        return;
    }

    // 准备用于回调的点云副本（如有需要）
    PointCloudMsgPtr local_cloud = cloud_callback_ ? std::make_shared<PointCloudMsg>(*curr_msg) : curr_msg;

    // 转换为PCL兼容点云（使用Eigen对齐分配器）
    using PCLPointVec = std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI>>;
    PCLPointVec temp_pcl_points;
    temp_pcl_points.reserve(local_cloud->points.size());

    std::transform(local_cloud->points.begin(), local_cloud->points.end(),
                   std::back_inserter(temp_pcl_points),
                   [](const type::PointXYZI& pt) -> pcl::PointXYZI {
                       return {pt.x, pt.y, pt.z, pt.intensity};
                   });

    // 更新PCL点云并刷新显示
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!viewer_ || viewer_->wasStopped()) {
            return;
        }

        pcl_cloud_->points = std::move(temp_pcl_points);
        pcl_cloud_->width = pcl_cloud_->points.size();
        pcl_cloud_->height = 1;

        viewer_->updatePointCloud<pcl::PointXYZI>(pcl_cloud_, *color_handler_, "rs_cloud");
        viewer_->spinOnce(10);
    }

    // 执行用户回调（在锁外执行，避免阻塞渲染）
    if (cloud_callback_) {
        cloud_callback_(local_cloud);
    }

#ifdef RS_TIME_RECORD
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
    RS_DEBUG << "viewer render time: " << duration << "us" << RS_REND;
#endif
}

void LidarViewer::keepWindowAlive() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (viewer_ && (state_ == ViewerState::INITIALIZED || state_ == ViewerState::RUNNING) && !viewer_->wasStopped()) {
        viewer_->spinOnce(10);
    }
}

bool LidarViewer::isWindowClosed() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return viewer_ && viewer_->wasStopped();
}

}  // namespace robosense::viewer