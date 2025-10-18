#include "rs_reader/LidarReader.h"
#include "rs_viewer/rs_viewer.h"
#include <iostream>
#include <thread>
#include <chrono>

using namespace robosense::type;
using namespace robosense::reader;
using namespace robosense::viewer;

int main(int argc, char** argv) {

    LidarReader reader("RSE1", "ONLINE_LIDAR", 6699, 7788);

    LidarViewer viewer("RoboSense Lidar Viewer");
    if (!viewer.init()) {
        std::cerr << "Failed to initialize LidarViewer!" << std::endl;
        reader.stop();
        return -1;
    }
    viewer.start();  // 切换到运行状态

    std::cout << "Start visualizing, close window to exit..." << std::endl;
    while (reader.isDriverRunning()) {
        // 检查窗口是否关闭（优先判断，避免无效点云处理）
        if (viewer.isWindowClosed()) {
            break;
        }

        // 获取点云（500ms超时，避免阻塞）
        PointCloudMsgPtr point_cloud = reader.getPointCloud(500000);
        if (point_cloud != nullptr) {
            // 处理并显示点云
            viewer.processAndShowPointCloud(point_cloud);
            // 释放原始点云（可视化器已处理，安全释放）
            reader.freePointCloud(point_cloud);
        } else {
            // 无新点云时保持窗口响应
            viewer.keepWindowAlive();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 降低CPU占用
        }
    }

    reader.stop();
    std::cout << "Viewer closed, program exited normally." << std::endl;
    return 0;
}