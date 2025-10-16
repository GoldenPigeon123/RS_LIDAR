#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <vector>
#include "rs_type/rs_point_cloud_type.h"
#include "rs_reader/LidarReader.h"

namespace py = pybind11;
namespace rs_type = robosense::type;
namespace rs_reader = robosense::reader;

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

/**
 * @brief 绑定LidarReader类到Python
 * 暴露激光雷达读取器的初始化、启动、获取点云等接口
 */
void bind_LidarReader(py::module& m) {
    py::class_<rs_reader::LidarReader>(m, "LidarReader")
        .def(py::init<const std::string&, const std::string&, uint16_t, uint16_t>(),
             py::arg("lidar_type_str") = "RSE1",
             py::arg("input_type_str") = "ONLINE_LIDAR",
             py::arg("msop_port") = 6699,
             py::arg("difop_port") = 7788)
        .def("set_pcap_path", &rs_reader::LidarReader::set_pcap_path,
             "设置PCAP文件路径（仅PCAP模式有效）")
        .def("set_distance_epsilon",&rs_reader::LidarReader::set_distance_epsilon,
            "设置欧式距离过滤器阈值")
        .def("regFilterFunction",&rs_reader::LidarReader::regFilterFunction,
            "注册自定义点云过滤器函数(Parameter:x,y,z)")
        .def("init", &rs_reader::LidarReader::init,
             "初始化驱动，返回是否成功")
        .def("start", &rs_reader::LidarReader::start,
             "启动数据读取，返回是否成功")
        .def("stop", &rs_reader::LidarReader::stop,
             "停止数据读取")
        .def("isDriverRunning", &rs_reader::LidarReader::isDriverRunning,
             "返回驱动是否正在运行")
        .def("getPointCloud", &rs_reader::LidarReader::getPointCloud,
             py::arg("usec") = 1000000,
             "获取点云数据，参数为超时时间（微秒）")
        .def("freePointCloud", &rs_reader::LidarReader::freePointCloud,
             "释放点云缓冲区")
        .def("getTemperature", &rs_reader::LidarReader::getTemperature,
             "获取激光雷达温度")
        .def("printDriverParam", &rs_reader::LidarReader::printDriverParam,
             "打印驱动参数配置")
        .def("printDeviceInfo", &rs_reader::LidarReader::printDeviceInfo,
             "打印设备信息")
        .def("printDeviceStatus", &rs_reader::LidarReader::printDeviceStatus,
             "打印设备状态");
}

/**
 * @brief Python模块入口
 * 定义rs_lidar模块，绑定核心数据类型和读取器类
 */
PYBIND11_MODULE(rs_lidar, m) {
    m.doc() = "RoboSense激光雷达Python接口（支持点云转Numpy数组）";
    bind_PointXYZI(m);
    bind_PointCloudMsg(m);
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