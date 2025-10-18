#ifndef RS_VIEWER_H_
#define RS_VIEWER_H_

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <memory>
#include <string>
#include <mutex>
#include <functional>
#include "rs_type/rs_point_cloud_type.h"

namespace robosense::viewer {

using PointCloudMsg = type::PointCloudT<type::PointXYZI>;
using PointCloudMsgPtr = std::shared_ptr<PointCloudMsg>;

/**
 * @enum ViewerState
 * @brief 状态枚举，表示可视化器的当前运行状态。
 */
enum class ViewerState {
    UNINITIALIZED,  ///< 未初始化状态
    INITIALIZED,    ///< 初始化完成，窗口已创建
    RUNNING         ///< 正在运行中
};

/**
 * @class LidarViewer
 * @brief LiDAR点云可视化类，基于PCL实现点云渲染与窗口管理。
 *
 * 支持强度着色显示、相机控制、回调扩展等功能。
 */
class LidarViewer {
public:
    /**
     * @brief 构造函数
     * @param window_name 可视化窗口名称，默认为 "Lidar Viewer"
     */
    explicit LidarViewer(const std::string& window_name = "Lidar Viewer");

    /**
     * @brief 析构函数
     * @note 自动清理资源并关闭窗口
     */
    ~LidarViewer();

    /**
     * @brief 初始化可视化器（创建窗口并配置参数）
     * @return 成功返回 true，失败或已初始化返回 false
     */
    bool init();

    /**
     * @brief 启动可视化器（切换至运行状态）
     * @return 成功返回 true，否则返回 false
     */
    bool start();

    /**
     * @brief 处理并显示一帧点云数据
     * @param curr_msg 当前点云消息指针
     */
    void processAndShowPointCloud(PointCloudMsgPtr curr_msg);

    /**
     * @brief 维持窗口响应（无点云输入时调用）
     */
    void keepWindowAlive();

    /**
     * @brief 检查窗口是否被用户关闭
     * @return 若窗口已关闭返回 true，否则返回 false
     */
    bool isWindowClosed() const;

    /**
     * @brief 注册点云处理回调函数
     * @param callback 回调函数对象，接收 PointCloudMsgPtr 类型参数
     */
    void registerPointCloudCallback(std::function<void(PointCloudMsgPtr)> callback);

private:
    std::string window_name_;  ///< 窗口名称
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;  ///< PCL可视化器实例
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> pcl_cloud_; ///< 内部PCL点云容器
    std::shared_ptr<pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>> color_handler_;  ///< 强度着色处理器
    ViewerState state_;                                          ///< 当前状态
    mutable std::mutex mutex_;                                   ///< 线程安全互斥锁
    std::function<void(PointCloudMsgPtr)> cloud_callback_;       ///< 用户注册的回调函数
};

}  // namespace robosense::viewer

#endif  // RS_VIEWER_H_