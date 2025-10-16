## 依赖声明
本项目基于 **RoboSense LiDAR SDK**（版权所有 © 2020 RoboSense）开发，遵循其3-clause BSD License协议。
使用前请确保已获取RoboSense SDK的合法授权（商业使用需联系RoboSense获取授权）。 
本项目基于rs_driver进行的小部分修改
### 原始SDK获取
- RoboSense官方SDK：[Robosense SDK](https://www.robosense.ai/developer)
- RoboSense官法rs_driver: [Rebosense rs_driver](https://github.com/RoboSense-LiDAR/rs_driver)
- SDK安装要求：详见RoboSense官方文档

### 原始SDK修改点

1. [src/rs_driver/driver/driver_param.hpp]("/usr/local/rs_driver/include/rs_driver/driver/driver_param.hpp")

```C++
inline InputType strToInputType(const std::string& type)
{
  static const std::unordered_map<std::string, InputType> strInputTypeMap = {
      {"ONLINE_LIDAR", InputType::ONLINE_LIDAR},
      {"PCAP_FILE", InputType::PCAP_FILE},
      {"RAW_PACKET", InputType::RAW_PACKET}
  };

  auto it = strInputTypeMap.find(type);
  if (it != strInputTypeMap.end()) {
      return it->second;
  } else {
    RS_ERROR << "Wrong input type: " << type << RS_REND;
    RS_ERROR << "Please give correct type: ONLINE_LIDAR, PCAP_FILE, RAW_PACKET." << RS_REND;
    exit(-1);
  }
}
```

```C++
struct DeviceInfo{
    void print(){
    RS_INFO << "------------------------------------------------------" << RS_REND;
    RS_INFO << "             RoboSense Device Info " << RS_REND;
    RS_INFOL << "state: " << state << RS_REND;
    RS_INFOL << "sn: " << sn << RS_REND;
    RS_INFOL << "mac: " << mac << RS_REND;

    RS_INFOL << "top_ver: " << top_ver << RS_REND;
    RS_INFOL << "bottom_ver: " << bottom_ver << RS_REND;
    RS_INFOL << "------------------------------------------------------" << RS_REND;
  }
}
```

```C++

struct DeviceStatus{
    void print(){
    RS_INFO << "------------------------------------------------------" << RS_REND;
    RS_INFO << "             RoboSense Device Status " << RS_REND;
    RS_INFOL << "state: " << state << RS_REND;
    RS_INFOL << "voltage: " << voltage << RS_REND;
    RS_INFO << "------------------------------------------------------" << RS_REND;
  };
}
```
