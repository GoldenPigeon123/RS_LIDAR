#include "rs_viewer/rs_viewer.h"
#include "rs_type/rs_point_cloud_type.h"
#include <pcl/console/print.h>  // 禁用PCL警告

namespace robosense::viewer {

LidarViewer::LidarViewer(const std::string& window_name)
    : window_name_(window_name), cloud_callback_(nullptr) {
    // 禁用PCL/VTK的警告输出
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

    // 初始化PCL点云容器
    pcl_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl_cloud_->height = 1;       // 无序点云
    pcl_cloud_->width = 0;
    pcl_cloud_->is_dense = true;

    // 初始化可视化器
    initViewer();
}

LidarViewer::~LidarViewer() {
    // 清理可视化资源
    if (viewer_) {
        viewer_->removeAllPointClouds();
        viewer_->removeAllCoordinateSystems();
        viewer_.reset();
    }
    pcl_cloud_->clear();
    color_handler_.reset();
    visualized_cloud_.reset();  // 释放副本
}

void LidarViewer::initViewer() {
    // 创建可视化窗口
    viewer_ = std::make_shared<pcl::visualization::PCLVisualizer>(window_name_);
    viewer_->setBackgroundColor(0.0, 0.0, 0.0);  // 黑色背景

    // 配置点显示属性
    viewer_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2  // 点大小为2
    );
    viewer_->addCoordinateSystem(1.0);  // 添加坐标系（轴长1米）

    // 初始化相机位置（从X轴负方向观察原点）
    viewer_->initCameraParameters();
    viewer_->setCameraPosition(
        -10.0, 0.0, 0.0,  // 相机位置
        0.0, 0.0, 0.0,    // 目标点
        0.0, 0.0, 1.0     // 上方向（Z轴）
    );

    // 初始化强度着色器（蓝→红渐变，强度越高越红）
    color_handler_ = std::make_shared<pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>>(
        pcl_cloud_, "intensity"
    );
}

void LidarViewer::registerPointCloudCallback(std::function<void(PointCloudMsgPtr)> callback) {
    cloud_callback_ = std::move(callback);  // 保存用户注册的回调
}

void LidarViewer::processAndShowPointCloud(PointCloudMsgPtr curr_msg) {
    if (!curr_msg || !viewer_) {
        return;  // 无效输入或未初始化，直接返回
    }

    // 1. 深复制点云（创建可视化专用副本，避免原始数据被修改）
    visualized_cloud_ = std::make_shared<PointCloudMsg>(*curr_msg);  // 利用复制构造函数

    // 2. 转换为PCL格式（用于可视化）
    pcl_cloud_->clear();
    for (const auto& pt : visualized_cloud_->points) {
        pcl::PointXYZI pcl_pt;
        pcl_pt.x = pt.x;
        pcl_pt.y = pt.y;
        pcl_pt.z = pt.z;
        pcl_pt.intensity = pt.intensity;  // 强度值用于着色
        pcl_cloud_->points.push_back(pcl_pt);
    }
    pcl_cloud_->width = pcl_cloud_->points.size();
    pcl_cloud_->height = 1;  // 保持无序点云

    // 3. 更新可视化显示（首次添加/后续更新）
    color_handler_->setInputCloud(pcl_cloud_);
    if (!viewer_->updatePointCloud<pcl::PointXYZI>(pcl_cloud_, *color_handler_, "rslidar_cloud")) {
        viewer_->addPointCloud<pcl::PointXYZI>(pcl_cloud_, *color_handler_, "rslidar_cloud");
    }

    // 4. 若注册了回调，调用回调函数（可用于内存回收、日志等）
    if (cloud_callback_) {
        cloud_callback_(visualized_cloud_);  // 传入可视化副本
    }

    viewer_->spinOnce(1);  // 处理窗口事件（1ms超时，不阻塞主线程）
}

void LidarViewer::keepWindowAlive() {
    if (viewer_ && !viewer_->wasStopped()) {
        viewer_->spinOnce(1);  // 无新点云时保持窗口响应
    }
}

bool LidarViewer::isWindowClosed() const {
    return (viewer_ && viewer_->wasStopped()) || !viewer_;
}

}  // namespace robosense::viewer