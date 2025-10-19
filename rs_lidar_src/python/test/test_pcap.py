import rs_lidar
import numpy as np

reader = rs_lidar.LidarReader("RSE1", "PCAP_FILE", 6699, 7788)
reader.set_pcap_path("./../../../../record/01.pcap")
reader.set_distance_epsilon(10.0)
reader.init()   
reader.start()

while reader.isDriverRunning():
    cloud = reader.getPointCloud()
    if cloud:
        # 直接转换为Numpy数组 [[X,Y,Z,I],...]
        points_np = cloud.to_numpy()
        print(f"帧 {cloud.seq} - 数组形状: {points_np.shape}")  # 输出 (N, 4)
        reader.freePointCloud(cloud)
    else:
        print("未获取到点云数据")
        reader.stop()

print("程序结束")

