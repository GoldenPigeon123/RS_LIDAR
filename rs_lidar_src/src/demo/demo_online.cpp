#include "rs_reader/LidarReader.h"
#include <iostream>
#include <thread>  

using namespace robosense::type;
using namespace robosense::reader;

int main(){
    // 1. 创建在线模式的阅读器
    // 参数: 雷达型号, 模式, MSOP端口, DIFOP端口
    LidarReader reader("RSE1", "ONLINE_LIDAR", 6699, 7788);
    
    // 2. 初始化并启动阅读器
    if (!reader.init()) {
        std::cerr << "[Error] LidarReader init failed!" << std::endl;
        return -1;
    }
    
    if (!reader.start()) {
        std::cerr << "[Error] LidarReader start failed!" << std::endl;
        return -1;
    }

    // 3. 循环获取点云
    while(true){
        PointCloudMsgPtr point_cloud = reader.getPointCloud();  
        
        if (point_cloud == nullptr) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));  
            continue;
        }

        std::cout << "[Info] Received point cloud: " << point_cloud->points.size() << " points" << std::endl;

        // 释放点云缓冲区
        reader.freePointCloud(point_cloud);
    }

    return 0;
}
