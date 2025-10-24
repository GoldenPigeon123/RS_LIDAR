#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <vector>
#include "rs_type/rs_point_cloud_type.h"
#include "rs_viewer/LidarViewer.h"

namespace py = pybind11;
namespace rs_type = robosense::type;
namespace rs_viewer = robosense::viewer;

void bind_LidarViewer(py::module& m) {
    py::class_<rs_viewer::LidarViewer>(m, "LidarViewer")
        .def(py::init<const std::string&>(),
            py::arg("window_name") = "Lidar Viewer",
            "构造函数，指定可视化窗口名称")
        .def("init", &rs_viewer::LidarViewer::init,
            "初始化可视化器（创建窗口并配置参数），成功返回True，失败或已初始化返回False")
        .def("start", &rs_viewer::LidarViewer::start,
            "启动可视化器（切换至运行状态），成功返回True，否则返回False")
        .def("processAndShowPointCloud", &rs_viewer::LidarViewer::processAndShowPointCloud,
            py::arg("curr_msg"),
            "处理并显示一帧点云数据（参数为PointCloudMsg的智能指针）")
        .def("keepWindowAlive", &rs_viewer::LidarViewer::keepWindowAlive,
            "维持窗口响应（无点云输入时调用）")
        .def("isWindowClosed", &rs_viewer::LidarViewer::isWindowClosed,
            "检查窗口是否被关闭，返回True表示已关闭");
}
