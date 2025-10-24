#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <vector>

#include "rs_bind/rs_bind_rs_type.h"
#include "rs_bind/rs_bind_rs_reader.h"
#include "rs_bind/rs_viewer.h"

namespace py = pybind11;

/**
 * @brief Python模块入口
 * 定义rs_lidar模块，绑定核心数据类型、读取器类和可视化类
 */
PYBIND11_MODULE(rs_lidar, m) {
    m.doc() = "RoboSense激光雷达Python接口（支持点云转Numpy数组）";
    bind_PointXYZI(m);
    bind_PointCloudMsg(m);
    bind_LidarViewer(m);
    bind_LidarReader(m);

    // 异常转换：将C++异常映射为Python异常
    py::register_exception_translator([](std::exception_ptr p) {
        try {
            if (p) std::rethrow_exception(p);
        } catch (const std::runtime_error& e) {
            PyErr_SetString(PyExc_RuntimeError, e.what());
        } catch (const std::invalid_argument& e) {
            PyErr_SetString(PyExc_ValueError, e.what());
        }
    });
}