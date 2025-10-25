## 联系我们
![ZUFE](./img/ZUFE.png)
- 学校：浙江财经大学
- 地址：浙江省杭州市下沙高教园区学源街18号
- 团队：浙江财经大学信息技术与人工智能学院点云组
- 邮箱：2789632062@qq.com
- GitHub: [https://github.com/GoldenPigeon123](https://gdithub.com/GoldenPigeon123)


## 附录

rs_driver_update基于[rs_driver](./../rs_driver_update/README_CN.md)进行了小部分修改，修改点如下：

### rs_driver修改点

1. [rs_driver/driver/driver_param.hpp](./../rs_driver_update/src/rs_driver/driver/driver_param.hpp)

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

