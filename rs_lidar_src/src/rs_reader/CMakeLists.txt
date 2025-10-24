cmake_minimum_required(VERSION 3.10)
project(rs_reader)

file(GLOB RS_READER_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_library(rs_reader STATIC ${RS_READER_SRC})
target_include_directories(rs_reader PUBLIC 
  ${rs_driver_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(rs_reader PUBLIC 
  ${rs_driver_LIBRARIES}
  pthread  # 多线程库
)

if(EXECUTIONLIB STREQUAL ON)
  target_link_libraries(rs_reader PRIVATE TBB::tbb)  # TBB库
endif()