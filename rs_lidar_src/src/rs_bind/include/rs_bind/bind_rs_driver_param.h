#include <string>
#include <pybind11/pybind11.h>

#include <rs_driver/driver/driver_param.hpp>

namespace rs_lidar =robosense::lidar;
namespace py = pybind11;

py::dict RSDriverParam_to_dict(const rs_lidar::RSDriverParam &param){
    py::dict dict;
    dict["lidar_type"]=lidarTypeToStr(param.lidar_type);
    dict["input_type"]=inputTypeToStr(param.input_type);
    dict["frame_id"]=param.frame_id;

    return dict;
};
py::dict DeviceInfo_to_dict(const rs_lidar::DeviceInfo &info){
    py::dict dict;
    dict["state"]=info.state;
    dict["sn"]=info.sn;
    dict["mac"]=info.mac;
    dict["top_ver"]=info.top_ver;
    dict["bottom_ver"]=info.bottom_ver;

    dict["qx"]=info.qx;
    dict["qy"]=info.qy;
    dict["qz"]=info.qz;
    dict["qw"]=info.qw;
    dict["x"]=info.x;
    dict["y"]=info.y;
    dict["z"]=info.z;

    return dict;
}

py::dict DeviceStatus_to_dict(const rs_lidar::DeviceStatus &status){
    py::dict dict;
    dict["voltage"]=status.voltage;
    dict["state"]=status.state;

    return dict;
}