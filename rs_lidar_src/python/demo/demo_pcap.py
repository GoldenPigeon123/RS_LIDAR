import rs_lidar
import signal
import sys
import numpy as np

# 全局变量用于资源管理
reader = None
viewer = None

def signal_handler(signum, frame):
    """信号处理函数，用于响应 Ctrl+C"""
    print("\n正在优雅退出...")
    if reader:
        reader.stop()
    print("资源已释放，程序退出。")
    sys.exit(0)

def main():
    global reader, viewer

    # 注册信号处理器
    signal.signal(signal.SIGINT, signal_handler)

    try:
        # 初始化 LidarReader
        reader = rs_lidar.LidarReader("RSE1", "PCAP_FILE", 6699, 7788)
        reader.set_pcap_path("./../../../../record/01.pcap")
        # reader.set_pcap_path("./../../../data/outdoor.pcap")
        # reader.set_pcap_path("./../../../data/indoor.pcap")
        reader.set_distance_epsilon(12.0)
        reader.init()
        reader.start()

        # 初始化 LidarViewer
        viewer = rs_lidar.LidarViewer("RSE1 Viewer")
        viewer.setRenderMode("RGB")
        viewer.init()
        viewer.start()

        print("开始播放点云数据，按 Ctrl+C 退出...")

        # 主循环：持续获取并显示点云
        while reader.isDriverRunning():
            cloud = reader.getPointCloud()
            if cloud:
                cloud_np=cloud.to_numpy()
                z_values=cloud_np[:,2]
                mask=z_values<-0.8
                indices = np.where(mask)[0]
                viewer.addPointCloud(cloud)
                viewer.setPointColor(indices, "red")
                viewer.show()
                reader.freePointCloud(cloud)
            else:
                print("未获取到点云数据，准备退出...")
                break

    except Exception as e:
        print(f"程序运行出错: {e}")
    finally:
        # 确保资源释放
        print("正在关闭程序...")
        # reader.stop()
            

if __name__ == "__main__":
    main()
    print("程序已安全退出。")