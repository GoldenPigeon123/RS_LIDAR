import rs_lidar

reader = rs_lidar.LidarReader("RSE1", "PCAP_FILE", 6699, 7788)
reader.set_pcap_path("./../../../data/outdoor.pcap")
reader.set_distance_epsilon(10.0)
reader.init()   
reader.start()

viewer = rs_lidar.LidarViewer("RSE1 Viewer")
viewer.init()
viewer.start()

while reader.isDriverRunning():
    cloud = reader.getPointCloud()
    if cloud:
        viewer.processAndShowPointCloud(cloud)
        reader.freePointCloud(cloud)
    else:
        reader.stop()

print("程序结束")

