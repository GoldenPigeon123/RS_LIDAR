##  版权声明

本仓库中的代码为浙江财经大学9106实验室点云组基于[rs_driver](https://github.com/RoboSense-LiDAR/rs_driver/releases)的二次开发。

![RobosenseLogo](./doc/img/RobosenseLogo.png)

本产品/软件集成了 RoboSense LiDAR SDK。该软件由 RoboSense 提供，并遵循 BSD 3-Clause 许可证。

<img src="/doc/video/demo.gif" alt="点云分割演示" width="100%" />


## 相关依赖

### Python3.10
编译安装包的时候请采用Python3.10版本，其他版本可能会出现编译错误。

### PCL1.15+VTK9.1

#### [PCL1.15](https://github.com/PointCloudLibrary/pcl)

请你选择特定的目录进行安装


```bash
git clone https://github.com/PointCloudLibrary/pcl.git
cd pcl
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_GPU=ON && make -j4
```

#### VTK9.1

```bash
sudo apt-get install libvtk9-dev
```

## 本项目

阅读顺序

1.[compile.md](./doc/compile.md)

2.[usage.md](./doc/usage.md)

3.[addition.md](./doc/addition.md)