#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "rs_type/rs_point_cloud_type.h"
#include "rs_viewer/LidarViewer.h"

namespace py = pybind11;
namespace rs_viewer = robosense::viewer;

void bind_LidarViewer(py::module& m) {
    py::class_<rs_viewer::LidarViewer>(m, "LidarViewer")
        .def(py::init<const std::string&>(), py::arg("window_name") = "Lidar Viewer")
        .def("set_render_mode", &rs_viewer::LidarViewer::setRenderMode, py::arg("mode"))
        .def("init", &rs_viewer::LidarViewer::init)
        .def("start", &rs_viewer::LidarViewer::start)
        .def("add_point_cloud", &rs_viewer::LidarViewer::addPointCloud, py::arg("curr_msg"))
        .def("show", &rs_viewer::LidarViewer::show)
        .def("keep_window_alive", &rs_viewer::LidarViewer::keepWindowAlive)
        .def("is_window_closed", &rs_viewer::LidarViewer::isWindowClosed);
}