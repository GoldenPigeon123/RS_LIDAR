#ifndef RS_VIEWER_H_
#define RS_VIEWER_H_

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <memory>
#include <string>
#include <functional>
#include <mutex>  // 线程安全互斥锁
#include "rs_type/rs_point_cloud_type.h"  // 你的点云类型定义

namespace robosense::viewer {

using PointCloudMsg = type::PointCloudT<type::PointXYZI>;  // 简化类型别名
using PointCloudMsgPtr = std::shared_ptr<PointCloudMsg>;

class LidarViewer {
public:
    /**
     * @brief 构造函数，初始化可视化窗口
     * @param window_name 窗口标题
     */
    explicit LidarViewer(const std::string& window_name = "RoboSense Lidar Viewer");

    /**
     * @brief 析构函数，释放资源
     */
    ~LidarViewer();

    /**
     * @brief 注册点云回调函数（可选，用于后续处理）
     * @param callback 回调函数，参数为可视化用的点云副本
     */
    void registerPointCloudCallback(std::function<void(PointCloudMsgPtr)> callback);

    /**
     * @brief 处理并显示点云（核心接口）
     * @param curr_msg 原始点云消息（会被深复制后再处理）
     */
    void processAndShowPointCloud(PointCloudMsgPtr curr_msg);

    /**
     * @brief 无新点云时保持窗口响应
     */
    void keepWindowAlive();

    /**
     * @brief 判断窗口是否已关闭
     * @return 窗口关闭返回true，否则false
     */
    bool isWindowClosed() const;

private:
    /**
     * @brief 初始化可视化器（内部调用）
     */
    void initViewer();

    std::string window_name_;  ///< 窗口标题
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;  ///< PCL可视化器
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> pcl_cloud_;  ///< PCL格式点云（用于可视化）
    std::shared_ptr<pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>> color_handler_;  ///< 强度着色器
    PointCloudMsgPtr visualized_cloud_;  ///< 可视化用的点云副本（深复制）
    std::function<void(PointCloudMsgPtr)> cloud_callback_;  ///< 用户注册的回调函数
    mutable std::mutex viewer_mutex_;  ///< 保护可视化器的线程安全锁（mutable允许const函数使用）
};

}  // namespace robosense::viewer

#endif  // RS_VIEWER_H_