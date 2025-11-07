#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "rs_type/rs_point_cloud_type.h"
#include "rs_viewer/LidarViewer.h"

namespace py = pybind11;
namespace rs_viewer = robosense::viewer;

void bind_LidarViewer(py::module& m) {
    py::class_<rs_viewer::LidarViewer>(m, "LidarViewer")
        .def(py::init<const std::string&>(), py::arg("window_name") = "Lidar Viewer")
        .def("setRenderMode", &rs_viewer::LidarViewer::setRenderMode, py::arg("mode"))
        .def("init", &rs_viewer::LidarViewer::init,"Initialize the lidar viewer. Must be called after setRenderMode().\n",
            "Returns: bool - True if initialization succeeds, False otherwise.")
        .def("start", &rs_viewer::LidarViewer::start)
        .def("addPointCloud", &rs_viewer::LidarViewer::addPointCloud, py::arg("curr_msg"))
        .def("setPointColor",&rs_viewer::LidarViewer::setPointColor,py::arg("indices"),py::arg("color"))
        .def("show", &rs_viewer::LidarViewer::show)
        .def("keepWindowAlive", &rs_viewer::LidarViewer::keepWindowAlive)
        .def("isWindowClosed", &rs_viewer::LidarViewer::isWindowClosed)
        .def("getRenderMode", &rs_viewer::LidarViewer::getRenderMode)
        .def("add_xyz_numpy", [](rs_viewer::LidarViewer& self, py::array_t<float> arr) {
            // 校验维度：必须为2维
            if (arr.ndim() != 2) {
                throw std::invalid_argument("Numpy array must be 2-dimensional (shape: [N, 3])");
            }
            auto shape = arr.shape();
            // 校验列数：必须为3（XYZ）
            if (shape[1] != 3) {
                throw std::invalid_argument("Numpy array must have 3 columns (XYZ), got " + std::to_string(shape[1]));
            }

            // 校验渲染模式：必须为RGB
            if (self.getRenderMode() != "RGB") {
                throw std::runtime_error("XYZ numpy array requires 'RGB' render mode. Call setRenderMode('RGB') first.");
            }

            // 安全访问数组（自动处理内存对齐和步长）
            auto arr_unchecked = arr.unchecked<2>(); // 无边界检查，性能更优

            // 转换为PointCloudMsg（拷贝数据，避免内存映射问题）
            PointCloudMsgPtr msg = std::make_shared<PointCloudMsg>();
            msg->points.reserve(shape[0]); // 预分配内存

            for (size_t i = 0; i < shape[0]; ++i) {
                rs_type::PointXYZI pt;
                pt.x = arr_unchecked(i, 0);  // 安全访问X坐标
                pt.y = arr_unchecked(i, 1);  // 安全访问Y坐标
                pt.z = arr_unchecked(i, 2);  // 安全访问Z坐标
                pt.intensity = 0;            // XYZ模式下强度值无效，置0
                msg->points.push_back(pt);
            }

            self.addPointCloud(msg); // 复用现有添加逻辑
        }, py::arg("arr"), "Add XYZ point cloud from numpy array (shape: [N, 3])")
        ;
}