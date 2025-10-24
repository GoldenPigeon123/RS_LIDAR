cmake_minimum_required(VERSION 3.10)
project(rs_viewer)

file(GLOB RS_VIEWER_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_library(rs_viewer STATIC ${RS_VIEWER_SRC})
target_include_directories(rs_viewer PUBLIC 
  ${rs_driver_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(rs_viewer PUBLIC 
  ${rs_driver_LIBRARIES}
  pthread  # 多线程库
)