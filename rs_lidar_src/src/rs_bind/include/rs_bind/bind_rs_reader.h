#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <vector>
#include <string>
#include "bind_rs_driver_param.h"
#include "rs_type/rs_point_cloud_type.h"
#include "rs_reader/LidarReader.h"

namespace py = pybind11;
namespace rs_type = robosense::type;
namespace rs_reader = robosense::reader;

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
          .def("getDriverParam", [](rs_reader::LidarReader& self){
               return RSDriverParam_to_dict(self.getDriverParam());
          }
               ,"获取驱动参数配置")
          .def("printDeviceInfo", &rs_reader::LidarReader::printDeviceInfo,
               "打印设备信息")
          .def("getDeviceInfo", [](rs_reader::LidarReader& self){
               return  DeviceInfo_to_dict(self.getDeviceInfo());
          },
               "获取设备信息")
          .def("printDeviceStatus", &rs_reader::LidarReader::printDeviceStatus,
               "打印设备状态")
          .def("getDeviceStatus",[](rs_reader::LidarReader& self){
               return DeviceStatus_to_dict(self.getDeviceStatus());
          },
               "获取设备状态")
          ;
}