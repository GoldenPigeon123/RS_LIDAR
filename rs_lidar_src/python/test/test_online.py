import rs_lidar

reader = rs_lidar.LidarReader("RSE1", "ONLINE_LIDAR", 6699, 7788)
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

