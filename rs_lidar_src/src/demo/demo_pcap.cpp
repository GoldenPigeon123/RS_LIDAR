#include "rs_reader/LidarReader.h"
#include "rs_viewer/rs_viewer.h" 
#include <iostream>
#include <thread>  

using namespace robosense::type;
using namespace robosense::reader;
using namespace robosense::viewer;

int main(int argc, char**argv){
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <pcap_file_path>" << std::endl;
        return -1;
    }

    // 初始化阅读器和可视化器
    LidarReader reader("RSE1", "PCAP_FILE", 6699, 7788);
    reader.set_pcap_path(argv[1]);
    LidarViewer viewer("RoboSense Lidar PCAP Viewer");  // 窗口标题

    // 初始化并启动阅读器
    if (!reader.init() || !reader.start()) {
        return -1;
    }

    // 循环获取点云并显示（同时检查窗口是否关闭）
    while (reader.isDriverRunning() && !viewer.isWindowClosed()) {  // 增加窗口关闭检查
        // 获取点云（500ms超时）
        PointCloudMsgPtr point_cloud = reader.getPointCloud(500000);
        
        if (point_cloud == nullptr) {
            // 无新点云时，保持窗口响应（避免窗口卡死）
            viewer.keepWindowAlive();
            continue;  // 继续循环，直到窗口关闭
        }

        // 关键：将点云传递给可视化器显示（内部会自动深复制点云，不影响原始数据）
        viewer.processAndShowPointCloud(point_cloud);

        // 释放原始点云（可视化器已复制，可安全释放）
        reader.freePointCloud(point_cloud);
    }

    // 停止阅读器并退出
    reader.stop();
    std::cout << "Viewer closed, program exited" << std::endl;
    return 0;
}