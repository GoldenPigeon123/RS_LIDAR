#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <vector>
#include <chrono>
#include <rs_driver/common/rs_log.hpp>
#include "rs_type/rs_point_cloud_type.h"

namespace py = pybind11;
namespace rs_type = robosense::type;

/**
 * @brief 绑定PointXYZI结构体到Python
 * 暴露x、y、z坐标和intensity强度值
 */
void bind_PointXYZI(py::module& m) {
    py::class_<rs_type::PointXYZI>(m, "PointXYZI")
        .def(py::init<>())
        .def_readwrite("x", &rs_type::PointXYZI::x)
        .def_readwrite("y", &rs_type::PointXYZI::y)
        .def_readwrite("z", &rs_type::PointXYZI::z)
        .def_readwrite("intensity", &rs_type::PointXYZI::intensity)
        .def("__repr__", [](const rs_type::PointXYZI& p) {
            return py::str("PointXYZI(x={:.3f}, y={:.3f}, z={:.3f}, intensity={})")
                .format(p.x, p.y, p.z, static_cast<int>(p.intensity));
        });
}

/**
 * @brief 绑定PointCloudMsg类到Python
 * 暴露时间戳、帧序号、坐标系ID等元信息，提供to_numpy方法转换为Numpy数组
 */
void bind_PointCloudMsg(py::module& m) {
    using PointCloudMsg = rs_type::PointCloudMsg;
    using PointCloudMsgPtr = rs_type::PointCloudMsgPtr;

    py::class_<PointCloudMsg, PointCloudMsgPtr>(m, "PointCloudMsg")
        .def(py::init<>())
        .def_readwrite("timestamp", &PointCloudMsg::timestamp)  ///< 点云时间戳
        .def_readwrite("seq", &PointCloudMsg::seq)              ///< 帧序号
        .def_readwrite("frame_id", &PointCloudMsg::frame_id)    ///< 坐标系ID
        .def_readwrite("points", &PointCloudMsg::points)        ///< 点数据列表
        .def("to_numpy", [](const PointCloudMsg& cloud) {
#ifdef RS_TIME_RECORD
    auto start_time = std::chrono::high_resolution_clock::now();
#endif
             // 点数量（转换为Numpy兼容的索引类型）
            py::ssize_t n = static_cast<py::ssize_t>(cloud.points.size());

            // 定义Numpy数组形状：(点数量, 4)，对应XYZI四个维度
            std::vector<py::ssize_t> shape = {n, 4};

            // 定义步长（字节）：每行步长为4个float（16字节），每个元素步长为1个float（4字节）
            std::vector<py::ssize_t> strides = {4 * sizeof(float), sizeof(float)};

            // 创建Numpy数组（未初始化内存）
            py::array_t<float> arr(shape, strides);
            auto ptr = static_cast<float*>(arr.mutable_data());  // 获取数据指针

            // 复制点数据到Numpy数组
            for (py::ssize_t i = 0; i < n; ++i) {
                const auto& p = cloud.points[static_cast<size_t>(i)];
                ptr[4*i + 0] = p.x;
                ptr[4*i + 1] = p.y;
                ptr[4*i + 2] = p.z;
                ptr[4*i + 3] = p.intensity;
            }
    
#ifdef RS_TIME_RECORD
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
    RS_DEBUG << "to_numpy() takes " << duration << "us" << RS_REND;
#endif
        return arr;
        })
        .def("__repr__", [](const PointCloudMsg& cloud) {
            return py::str("PointCloudMsg(frame_id='{}', seq={}, points_count={})")
                .format(cloud.frame_id, cloud.seq, cloud.points.size());
        });
}
