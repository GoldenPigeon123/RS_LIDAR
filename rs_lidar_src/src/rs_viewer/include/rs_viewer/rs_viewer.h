#ifndef _RS_VIEWER_H_
#define _RS_VIEWER_H_

// 强制启用PCL点类型（可视化依赖PCL）
#ifndef USE_PCL_POINT_TYPE
#define USE_PCL_POINT_TYPE
#endif

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <memory>
#include <string>
#include <functional>  // 用于回调函数

// 自定义点云类型
#include "rs_type/rs_point_cloud_type.h"

using robosense::type::PointT;               ///< 点类型（XYZI）
using robosense::type::PointCloudMsg;        ///< 点云消息类型
using robosense::type::PointCloudMsgPtr;     ///< 点云智能指针

namespace robosense::viewer {

class LidarViewer {
public:
    /**
     * @brief 构造函数：仅需窗口名称初始化
     * @param window_name 可视化窗口标题（如"RoboSense Lidar Viewer"）
     */
    explicit LidarViewer(const std::string& window_name);

    /**
     * @brief 析构函数：清理可视化资源
     */
    ~LidarViewer();

    /**
     * @brief 注册点云回调函数（可选）
     * @param callback 回调函数，参数为当前处理的点云指针
     * @note 调试时可不注册（仅可视化），实际使用时注册用于后续处理（如内存回收）
     */
    void registerPointCloudCallback(std::function<void(PointCloudMsgPtr)> callback);

    /**
     * @brief 处理并显示点云
     * @param curr_msg 待显示的点云指针（会被深复制为可视化副本）
     */
    void processAndShowPointCloud(PointCloudMsgPtr curr_msg);

    /**
     * @brief 保持窗口响应（循环中调用）
     */
    void keepWindowAlive();

    /**
     * @brief 检查窗口是否关闭
     * @return 窗口关闭返回true，否则返回false
     */
    bool isWindowClosed() const;

private:
    /**
     * @brief 初始化可视化器（窗口、背景、相机等）
     */
    void initViewer();

    std::string window_name_;  ///< 窗口名称
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;  ///< PCL可视化器
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_;  ///< PCL格式点云容器
    std::shared_ptr<pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>> color_handler_;  ///< 强度着色器

    // 点云回调（可选，用户注册后生效）
    std::function<void(PointCloudMsgPtr)> cloud_callback_;
    // 当前可视化的点云副本（避免原始点云被修改影响显示）
    PointCloudMsgPtr visualized_cloud_;
};

}  // namespace robosense::viewer

#endif  // _RS_VIEWER_H_