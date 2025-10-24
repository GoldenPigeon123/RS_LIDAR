#include "rs_viewer/LidarViewer.h"
#include <pcl/console/print.h>
#include <Eigen/StdVector>
#include <rs_driver/common/rs_log.hpp>

namespace robosense::viewer {

LidarViewer::LidarViewer(const std::string& window_name)
    : window_name_(window_name),
      state_(ViewerState::UNINITIALIZED) {
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
}

LidarViewer::~LidarViewer() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (viewer_) {
        viewer_->removeAllPointClouds();
        viewer_->close();
    }
}

// 在init()前设置渲染模式
void LidarViewer::setRenderMode(const std::string& mode) {
    std::lock_guard<std::mutex> lock(mutex_);
    render_mode_ = mode;  // 不检查输入，默认用户输入"RGB"或"Intensity"
}

bool LidarViewer::init() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != ViewerState::UNINITIALIZED || render_mode_.empty()) {
        return false;  // 必须先设置模式
    }

    // 创建可视化窗口
    viewer_ = std::make_shared<pcl::visualization::PCLVisualizer>(window_name_);
    if (!viewer_ || viewer_->wasStopped()) {
        return false;
    }

    // 窗口基础配置
    viewer_->setBackgroundColor(0.0, 0.0, 0.0);
    viewer_->addCoordinateSystem(1.0);
    viewer_->initCameraParameters();
    viewer_->setCameraPosition(-10.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

    // 根据模式初始化对应资源
    if (render_mode_ == "Intensity") {
        intensity_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        intensity_cloud_->height = 1;
        intensity_cloud_->width = 0;
        intensity_handler_ = std::make_shared<pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>>(
            intensity_cloud_, "intensity");
        viewer_->addPointCloud<pcl::PointXYZI>(intensity_cloud_, *intensity_handler_, "cloud");
    } else {  // RGB模式
        rgb_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        rgb_cloud_->height = 1;
        rgb_cloud_->width = 0;
        rgb_handler_ = std::make_shared<pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>>(
            rgb_cloud_);
        viewer_->addPointCloud<pcl::PointXYZRGB>(rgb_cloud_, *rgb_handler_, "cloud");
    }

    // 设置点大小
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

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

// 添加点云：根据模式转换为对应PCL类型
void LidarViewer::addPointCloud(PointCloudMsgPtr curr_msg) {
    if (state_ != ViewerState::RUNNING || !curr_msg || curr_msg->points.empty()) {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (render_mode_ == "Intensity") {
        // 强度模式：转换为pcl::PointXYZI
        using PCLPointVec = std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI>>;
        PCLPointVec temp_pcl_points;
        temp_pcl_points.reserve(curr_msg->points.size());

        std::transform(curr_msg->points.begin(), curr_msg->points.end(),
                       std::back_inserter(temp_pcl_points),
                       [](const type::PointXYZI& pt) -> pcl::PointXYZI {
                           return {pt.x, pt.y, pt.z, pt.intensity};
                       });

        intensity_cloud_->points = std::move(temp_pcl_points);
        intensity_cloud_->width = intensity_cloud_->points.size();
    } else {
        // RGB模式：转换为pcl::PointXYZRGB（默认白色）
        using RGBPointVec = std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>>;
        RGBPointVec temp_rgb_points;
        temp_rgb_points.reserve(curr_msg->points.size());

        std::transform(curr_msg->points.begin(), curr_msg->points.end(),
                       std::back_inserter(temp_rgb_points),
                       [](const type::PointXYZI& pt) -> pcl::PointXYZRGB {
                           pcl::PointXYZRGB rgb_pt;
                           rgb_pt.x = pt.x;
                           rgb_pt.y = pt.y;
                           rgb_pt.z = pt.z;
                           rgb_pt.r = 255;  // 默认白色
                           rgb_pt.g = 255;
                           rgb_pt.b = 255;
                           return rgb_pt;
                       });

        rgb_cloud_->points = std::move(temp_rgb_points);
        rgb_cloud_->width = rgb_cloud_->points.size();
    }
}

// RGB模式下：设置指定索引点的颜色
void LidarViewer::setPointColor(const std::vector<size_t>& indices, const std::string& color_name) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (render_mode_ != "RGB" || !rgb_cloud_ || rgb_cloud_->points.empty()) {
        return;
    }

    Color color = Color::fromName(color_name);
    for (size_t idx : indices) {
        if (idx < rgb_cloud_->points.size()) {  // 只检查越界，不检查索引有效性
            rgb_cloud_->points[idx].r = color.r;
            rgb_cloud_->points[idx].g = color.g;
            rgb_cloud_->points[idx].b = color.b;
        }
    }
}

// 显示点云
void LidarViewer::show() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != ViewerState::RUNNING || !viewer_ || viewer_->wasStopped()) {
        return;
    }

    // 根据模式更新点云
    if (render_mode_ == "Intensity") {
        viewer_->updatePointCloud<pcl::PointXYZI>(intensity_cloud_, *intensity_handler_, "cloud");
    } else {
        viewer_->updatePointCloud<pcl::PointXYZRGB>(rgb_cloud_, *rgb_handler_, "cloud");
    }
    viewer_->spinOnce(1);  // 非阻塞刷新
}

void LidarViewer::keepWindowAlive() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == ViewerState::RUNNING && viewer_ && !viewer_->wasStopped()) {
        viewer_->spinOnce(10);
    }
}

bool LidarViewer::isWindowClosed() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return viewer_ && viewer_->wasStopped();
}

}  // namespace robosense::viewer