#include "rs_viewer/rs_viewer.h"
#include <pcl/console/print.h>  // 禁用PCL警告
#include <cmath>  // 用于检查NaN

namespace robosense::viewer {

LidarViewer::LidarViewer(const std::string& window_name)
    : window_name_(window_name), cloud_callback_(nullptr) {
    // 禁用PCL/VTK的警告输出（避免干扰）
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

    // 初始化PCL点云容器（无序点云）
    pcl_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl_cloud_->height = 1;
    pcl_cloud_->width = 0;
    pcl_cloud_->is_dense = true;  // 初始设为稠密点云

    // 初始化可视化器
    initViewer();
}

LidarViewer::~LidarViewer() {
    // 加锁保护资源释放
    std::lock_guard<std::mutex> lock(viewer_mutex_);

    // 清理可视化资源
    if (viewer_) {
        viewer_->removeAllPointClouds();  // 移除所有点云
        viewer_->removeAllCoordinateSystems();  // 移除坐标系
        viewer_.reset();  // 释放可视化器
    }

    // 释放点云容器
    if (pcl_cloud_) {
        pcl_cloud_->clear();
        pcl_cloud_.reset();
    }
    color_handler_.reset();
    visualized_cloud_.reset();
}

void LidarViewer::initViewer() {
    std::lock_guard<std::mutex> lock(viewer_mutex_);  // 加锁初始化

    // 创建可视化窗口
    viewer_ = std::make_shared<pcl::visualization::PCLVisualizer>(window_name_);
    if (!viewer_) {
        throw std::runtime_error("Failed to create PCLVisualizer!");  // 初始化失败抛异常
    }

    // 配置窗口属性
    viewer_->setBackgroundColor(0.0, 0.0, 0.0);  // 黑色背景（突出点云）
    viewer_->addCoordinateSystem(1.0);  // 添加坐标系（轴长1米，x红/y绿/z蓝）

    // 初始化相机位置（从前方10米处观察原点）
    viewer_->initCameraParameters();
    viewer_->setCameraPosition(
        -10.0, 0.0, 2.0,  // 相机位置（x=-10, y=0, z=2，稍高于地面）
        0.0, 0.0, 0.0,    // 目标点（原点）
        0.0, 0.0, 1.0     // 上方向（Z轴）
    );

    // 初始化强度着色器（根据intensity字段着色）
    color_handler_ = std::make_shared<pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>>(
        pcl_cloud_, "intensity"
    );
}

void LidarViewer::registerPointCloudCallback(std::function<void(PointCloudMsgPtr)> callback) {
    cloud_callback_ = std::move(callback);  // 保存用户回调（线程安全，函数对象可移动）
}

void LidarViewer::processAndShowPointCloud(PointCloudMsgPtr curr_msg) {
    // 1. 入参检查（快速返回，避免加锁开销）
    if (!curr_msg || curr_msg->points.empty() || !viewer_) {
        return;
    }

    // 2. 加锁保护所有可视化操作（确保线程安全）
    std::lock_guard<std::mutex> lock(viewer_mutex_);

    // 3. 深复制点云（创建可视化专用副本，不影响原始数据）
    visualized_cloud_ = std::make_shared<PointCloudMsg>(*curr_msg);

    // 4. 转换为PCL格式（过滤无效点）
    pcl_cloud_->clear();
    pcl_cloud_->reserve(visualized_cloud_->points.size());  // 预分配内存

    for (const auto& pt : visualized_cloud_->points) {
        // 过滤无效点（NaN或无穷大坐标，避免渲染崩溃）
        if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z) ||
            std::isinf(pt.x) || std::isinf(pt.y) || std::isinf(pt.z)) {
            continue;
        }

        // 转换为PCL点类型
        pcl::PointXYZI pcl_pt;
        pcl_pt.x = pt.x;
        pcl_pt.y = pt.y;
        pcl_pt.z = pt.z;
        pcl_pt.intensity = pt.intensity;  // 强度值用于着色
        pcl_cloud_->points.push_back(pcl_pt);
    }

    // 更新PCL点云的尺寸信息
    pcl_cloud_->width = pcl_cloud_->points.size();
    pcl_cloud_->height = 1;
    pcl_cloud_->is_dense = (pcl_cloud_->points.size() == visualized_cloud_->points.size());  // 标记是否有无效点被过滤

    // 5. 更新可视化显示（首次添加/后续更新）
    color_handler_->setInputCloud(pcl_cloud_);  // 更新着色器的输入点云

    // 尝试更新点云，若失败则首次添加
    if (!viewer_->updatePointCloud<pcl::PointXYZI>(pcl_cloud_, *color_handler_, "rslidar_cloud")) {
        // 首次添加点云，并设置点大小（明确指定ID）
        viewer_->addPointCloud<pcl::PointXYZI>(pcl_cloud_, *color_handler_, "rslidar_cloud");
        viewer_->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,  // 点大小为2像素
            "rslidar_cloud"  // 与添加点云时的ID一致
        );
    }

    // 6. 调用用户注册的回调（若有）
    if (cloud_callback_) {
        cloud_callback_(visualized_cloud_);  // 传入可视化用的点云副本
    }

    // 7. 处理窗口事件（1ms超时，不阻塞主线程）
    viewer_->spinOnce(1);
}

void LidarViewer::keepWindowAlive() {
    std::lock_guard<std::mutex> lock(viewer_mutex_);  // 加锁保护
    if (viewer_ && !viewer_->wasStopped()) {
        viewer_->spinOnce(1);  // 处理窗口事件，保持响应
    }
}

bool LidarViewer::isWindowClosed() const {
    std::lock_guard<std::mutex> lock(viewer_mutex_);  // 加锁读取
    return (viewer_ && viewer_->wasStopped()) || !viewer_;
}

}  // namespace robosense::viewer