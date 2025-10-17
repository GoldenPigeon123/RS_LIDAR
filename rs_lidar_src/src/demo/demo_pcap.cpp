#include "rs_reader/LidarReader.h"
#include <iostream>
#include <thread>  

using namespace robosense::type;
using namespace robosense::reader;

int main(int argc, char**argv){
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <pcap_file_path>" << std::endl;
        return -1;
    }

    // 1. 创建PCAP模式的阅读器
    // 参数: 雷达型号, 模式, MSOP端口, DIFOP端口
    LidarReader reader("RSE1", "PCAP_FILE", 6699, 7788);
    
    // 2. 设置PCAP文件路径（PCAP模式特有）
    reader.set_pcap_path(argv[1]);
    
    // 3. 初始化并启动阅读器
    if (!reader.init()) {
        return -1;
    }
    
    if (!reader.start()) {
        return -1;
    }

    // 4. 循环获取点云
    int count = 0;
    while(reader.isDriverRunning()){
        PointCloudMsgPtr point_cloud = reader.getPointCloud(500000);  // 500ms超时
        
        if (point_cloud == nullptr) {
            std::cout << "[INFO] No more point clouds, exiting..." << std::endl;
            break;
        }

        // 释放点云缓冲区
        reader.freePointCloud(point_cloud);
    }

    // 停止阅读器
    reader.stop();
    return 0;
}
