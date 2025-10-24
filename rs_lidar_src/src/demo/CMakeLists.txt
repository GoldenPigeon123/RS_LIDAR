cmake_minimum_required(VERSION 3.10)
project(demo)

# 编译demo_pcap可执行程序（回放模式）
add_executable(demo_pcap ${CMAKE_CURRENT_SOURCE_DIR}/src/demo_pcap.cpp)
target_link_libraries(demo_pcap PUBLIC 
    rs_reader  # 依赖核心库rs_reader
    rs_viewer  # 依赖可视化库rs_viewer
    pthread
)

# 编译demo_online可执行程序（在线模式）
add_executable(demo_online ${CMAKE_CURRENT_SOURCE_DIR}/src/demo_online.cpp)
target_link_libraries(demo_online PUBLIC 
    rs_reader  # 依赖核心库rs_reader
    rs_viewer  # 依赖可视化库rs_viewer
    pthread
)