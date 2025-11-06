#ifndef RS_VIEWER_LIDAR_VIEWER_H_
#define RS_VIEWER_LIDAR_VIEWER_H_

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <memory>
#include <string>
#include <mutex>
#include <vector>
#include "rs_type/rs_point_cloud_type.h"
#include "rs_viewer/Color.h"

namespace robosense::viewer {

using PointCloudMsg = type::PointCloudT<type::PointXYZI>;
using PointCloudMsgPtr = std::shared_ptr<PointCloudMsg>;

enum class ViewerState {
    UNINITIALIZED,
    INITIALIZED,
    RUNNING
};

class LidarViewer {
public:
    explicit LidarViewer(const std::string& window_name = "Lidar Viewer");
    ~LidarViewer();

    // 在init()前设置渲染模式（"RGB"或"Intensity)
    void setRenderMode(const std::string& mode);

    const std::string& getRenderMode() const { return render_mode_; }

    // 初始化可视化器（需在setRenderMode后调用）
    bool init();

    // 启动可视化器
    bool start();

    // 添加点云（根据当前模式转换类型）
    void addPointCloud(PointCloudMsgPtr curr_msg);

    // RGB模式下：设置指定索引点的颜色（需在show()前调用）
    void setPointColor(const std::vector<size_t>& indices, const std::string& color_name);

    // 显示点云
    void show();

    // 维持窗口响应
    void keepWindowAlive();

    // 检查窗口是否关闭
    bool isWindowClosed() const;

private:
    std::string window_name_;
    std::string render_mode_;  // "RGB"或"Intensity"
    ViewerState state_;
    mutable std::mutex mutex_;

    // PCL可视化器
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;

    // 强度模式资源
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> intensity_cloud_;
    std::shared_ptr<pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>> intensity_handler_;

    // RGB模式资源
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> rgb_cloud_;
    std::shared_ptr<pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>> rgb_handler_;
};

}  // namespace robosense::viewer

#endif  // RS_VIEWER_LIDAR_VIEWER_H_