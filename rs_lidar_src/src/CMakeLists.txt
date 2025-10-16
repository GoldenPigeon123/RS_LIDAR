cmake_minimum_required(VERSION 3.10)
project(rs_lidar VERSION 1.0.0 LANGUAGES CXX)

# ==================== 基础编译配置 ====================
# 编译标准设置
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 字符集设置（解决中文显示问题）
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-finput-charset=UTF-8)
endif()

# 输出目录配置
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

# 构建类型配置（默认Release）
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING 
        "Choose the build type: Debug Release" FORCE)
endif()

# 根据构建类型设置编译选项
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    # Debug模式：无优化 + 完整调试信息
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g")
    message(STATUS "Build Mode: Debug (-O0 + 完整调试信息)")
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    # Release模式：-O3极致优化，关闭调试信息
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -g0")
    message(STATUS "Build Mode: Release (默认，-O3 极致优化)")
endif()


# ==================== 编译选项开关 ====================
option(USE_PCL_POINT_TYPE "Use PCL point cloud type instead of custom type" OFF)
option(EXECUTIONLIB "Include execution in .cpp" OFF)
option(PRINT_PARAMETER "cout parameters of lidar to console" ON)
option(PRINT_DEBUG "cout debug to console" ON)
option(PRINT_MSG "cout brief message of every frames to console when 'getPointCloud' call" ON)
option(RS_TIME_RECORD "cout time record to console" ON)

# PCL相关配置
if(USE_PCL_POINT_TYPE STREQUAL ON)
  message(STATUS "Using PCL point cloud type")
  find_package(PCL REQUIRED)
  if(NOT PCL_FOUND)
    message(FATAL_ERROR "PCL library not found. Please install PCL first.")
  endif()
  add_definitions(${PCL_DEFINITIONS})
  add_definitions(-DEIGEN_MAX_ALIGN_BYTES=32)  # 消除Eigen对齐警告
  add_definitions(-DUSE_PCL_POINT_TYPE)        # 使用PCL点云类型
else()
  message(STATUS "Using custom point cloud type")
endif()

# 其他编译选项
if(EXECUTIONLIB STREQUAL ON)
  message(STATUS "Include execution in .cpp")
  add_definitions(-DEXECUTIONLIB)
endif()

if(PRINT_PARAMETER STREQUAL ON)
  message(STATUS "cout parameters of lidar to console")
  add_definitions(-DPRINT_PARAMETER)
endif()

if(PRINT_DEBUG  STREQUAL ON)
  message(STATUS "cout status ,parameters and other information to console")
  add_definitions(-DPRINT_DEBUG)
endif()

if(PRINT_MSG STREQUAL ON)
  message(STATUS "cout brief message of every frames to console when 'getPointCloud' call")
  add_definitions(-DPRINT_MSG)
endif()


# ==================== 依赖库查找 ====================
find_package(rs_driver REQUIRED)
if(NOT rs_driver_FOUND)
  message(FATAL_ERROR "rs_driver not found. Please install it first.")
endif()


# ==================== 全局包含目录 ====================
include_directories(
  ${rs_driver_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/rs_type/include  # rs_type头文件目录
)

if(USE_PCL_POINT_TYPE)
  include_directories(${PCL_INCLUDE_DIRS})
endif()


# ==================== 目标编译 ====================
# 编译rs_reader库（核心激光雷达读取库）
file(GLOB RS_READER_SRC ${CMAKE_CURRENT_SOURCE_DIR}/rs_reader/src/*.cpp)
add_library(rs_reader STATIC ${RS_READER_SRC})
target_include_directories(rs_reader PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/rs_reader/include
  ${CMAKE_CURRENT_SOURCE_DIR}/rs_type/include
)
target_link_libraries(rs_reader PUBLIC 
  ${rs_driver_LIBRARIES}
  ${PCL_LIBRARIES}
  pthread  # 多线程库
  # TBB::tbb # TBB库
)

# 编译demo_pcap可执行程序（回放模式）
file(GLOB DEMO_PCAP_FILE ${CMAKE_CURRENT_SOURCE_DIR}/demo/demo_pcap.cpp)
add_executable(demo_pcap ${DEMO_PCAP_FILE})
target_include_directories(demo_pcap PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/rs_type/include
  ${CMAKE_CURRENT_SOURCE_DIR}/rs_reader/include
)
target_link_libraries(demo_pcap PUBLIC 
  rs_reader  # 依赖核心库rs_reader
  ${rs_driver_LIBRARIES}
  pthread
)
if(USE_PCL_POINT_TYPE)
  target_link_libraries(demo_pcap PUBLIC ${PCL_LIBRARIES})
endif()

# 编译demo_online可执行程序（在线模式）
file(GLOB DEMO_ONLINE_FILE ${CMAKE_CURRENT_SOURCE_DIR}/demo/demo_online.cpp)
add_executable(demo_online ${DEMO_ONLINE_FILE})
target_include_directories(demo_online PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/rs_type/include
  ${CMAKE_CURRENT_SOURCE_DIR}/rs_reader/include
)
target_link_libraries(demo_online PUBLIC 
  rs_reader  # 依赖核心库rs_reader
  ${rs_driver_LIBRARIES}
  pthread
)
if(USE_PCL_POINT_TYPE)
  target_link_libraries(demo_online PUBLIC ${PCL_LIBRARIES})
endif()


# ==================== 安装规则 ====================
# 1. 安装头文件
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/rs_reader/include/
        DESTINATION include/rs_reader
        FILES_MATCHING PATTERN "*.h"
        )
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/rs_type/include/
        DESTINATION include/rs_reader/rs_type
        FILES_MATCHING PATTERN "*.h"
        )

# 2. 安装静态库
install(TARGETS rs_reader
        ARCHIVE DESTINATION lib
        )

# 3. 生成CMake配置文件
# 创建目标导出
export(TARGETS rs_reader
       FILE rs_readerTargets.cmake
       NAMESPACE rs_reader::
       )

# 生成rs_readerConfig.cmake
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/rs_readerConfig.cmake.in
    ${CMAKE_BINARY_DIR}/rs_readerConfig.cmake
    @ONLY
)
install(FILES ${CMAKE_BINARY_DIR}/rs_readerConfig.cmake
        DESTINATION lib/cmake/rs_reader
        )

# 生成版本文件
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/rs_readerConfigVersion.cmake.in
    ${CMAKE_BINARY_DIR}/rs_readerConfigVersion.cmake
    @ONLY
)
install(FILES ${CMAKE_BINARY_DIR}/rs_readerConfigVersion.cmake
        DESTINATION lib/cmake/rs_reader
        )