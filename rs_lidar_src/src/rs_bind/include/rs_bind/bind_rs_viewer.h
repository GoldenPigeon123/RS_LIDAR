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
        .def("init", &rs_viewer::LidarViewer::init)
        .def("start", &rs_viewer::LidarViewer::start)
        .def("addPointCloud", &rs_viewer::LidarViewer::addPointCloud, py::arg("curr_msg"))
        .def("setPointColor",&rs_viewer::LidarViewer::setPointColor,py::arg("indices"),py::arg("color"))
        .def("show", &rs_viewer::LidarViewer::show)
        .def("keepWindowAlive", &rs_viewer::LidarViewer::keepWindowAlive)
        .def("isWindowClosed", &rs_viewer::LidarViewer::isWindowClosed)
        .def("getRenderMode", &rs_viewer::LidarViewer::getRenderMode)
        .def("add_numpy_array", [](rs_viewer::LidarViewer& self, py::array_t<float> arr) {
            // 校验Numpy数组维度
            auto buf = arr.request();
            if (buf.ndim != 2) {
                throw std::invalid_argument("Numpy array must be 2-dimensional (shape: [N, 3] or [N, 4])");
            }
            size_t cols = buf.shape[1];
            if (cols != 3 && cols != 4) {
                throw std::invalid_argument("Numpy array must have 3 (XYZ) or 4 (XYZI) columns");
            }

            // 处理模式约束：N*3只能用RGB模式
            bool is_xyzi = (cols == 4);
            if (!is_xyzi) {
                if (self.getRenderMode() != "RGB") {
                    throw std::runtime_error("N*3 (XYZ) array requires 'RGB' render mode. Use setRenderMode('RGB') first.");
                }
            }

            // 转换Numpy数组到PointCloudMsg
            PointCloudMsgPtr msg = std::make_shared<PointCloudMsg>();
            msg->points.reserve(buf.shape[0]);
            float* data = static_cast<float*>(buf.ptr);

            for (size_t i = 0; i < buf.shape[0]; ++i) {
                rs_type::PointXYZI pt;
                pt.x = data[i * cols + 0];  // X坐标
                pt.y = data[i * cols + 1];  // Y坐标
                pt.z = data[i * cols + 2];  // Z坐标
                pt.intensity = is_xyzi ? static_cast<uint8_t>(data[i * cols + 3]) : 0;  // 强度（N*3时设为0）
                msg->points.push_back(pt);
            }

            // 调用现有方法添加点云
            self.addPointCloud(msg);
        }, py::arg("arr"), "Add point cloud from numpy array (N*3 XYZ or N*4 XYZI)")
        ;
}