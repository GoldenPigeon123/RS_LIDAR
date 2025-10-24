#include "rs_reader/LidarReader.h"
#include "rs_viewer/LidarViewer.h"
#include <iostream>
#include <thread>
#include <chrono>

using namespace robosense::type;
using namespace robosense::reader;
using namespace robosense::viewer;

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <pcap_file_path> [render_mode: RGB/Intensity]" << std::endl;
        return -1;
    }
    std::string render_mode = (argc >= 3) ? argv[2] : "Intensity";

    LidarReader reader("RSE1", "PCAP_FILE", 6699, 7788);
    reader.set_pcap_path(argv[1]);
    if (!reader.init() || !reader.start()) {
        std::cerr << "Failed to start LidarReader!" << std::endl;
        return -1;
    }

    LidarViewer viewer("RoboSense Lidar Viewer");
    viewer.setRenderMode(render_mode);
    if (!viewer.init()) {
        std::cerr << "Failed to initialize LidarViewer!" << std::endl;
        reader.stop();
        return -1;
    }
    viewer.start();

    while (reader.isDriverRunning()) {
        if (viewer.isWindowClosed()) break;

        PointCloudMsgPtr point_cloud = reader.getPointCloud(500000);
        if (point_cloud != nullptr && !point_cloud->points.empty()) {
            viewer.addPointCloud(point_cloud);
            viewer.show();
            reader.freePointCloud(point_cloud);
        } else {
            viewer.keepWindowAlive();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    reader.stop();
    return 0;
}