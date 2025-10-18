import rs_lidar
import time

def main():
    # 直接在代码内设置PCAP文件路径（根据实际路径修改）
    pcap_path = "./../../../../record/01.pcap"  # 这里替换为你的PCAP文件路径

    # 1. 初始化雷达阅读器
    reader = rs_lidar.LidarReader(
        lidar_type_str="RSE1",
        input_type_str="PCAP_FILE",
        msop_port=6699,
        difop_port=7788
    )
    reader.set_pcap_path(pcap_path)

    # 初始化并启动阅读器
    if not reader.init() or not reader.start():
        print("雷达阅读器初始化失败！")
        return

    # 2. 初始化可视化器
    viewer = rs_lidar.LidarViewer("激光雷达点云可视化")
    if not viewer.init():
        print("可视化器初始化失败！")
        reader.stop()
        return
    viewer.start()

    # 3. 主循环：读取并显示点云
    print("开始可视化，关闭窗口退出程序...")
    try:
        while reader.isDriverRunning():
            # 检查窗口是否关闭
            if viewer.isWindowClosed():
                break

            # 获取点云（超时500ms）
            cloud = reader.getPointCloud(500000)
            if cloud:
                # 显示点云
                viewer.processAndShowPointCloud(cloud)
                # 释放资源
                reader.freePointCloud(cloud)
            else:
                # 无点云时保持窗口响应
                viewer.keepWindowAlive()
                time.sleep(0.5)  # 降低CPU占用

    except KeyboardInterrupt:
        print("\n用户中断程序")
    finally:
        # 清理资源
        # reader.stop()
        print("程序正常退出")

if __name__ == "__main__":
    main()