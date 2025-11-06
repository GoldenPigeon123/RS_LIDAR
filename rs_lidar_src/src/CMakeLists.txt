cmake_minimum_required(VERSION 3.10)
project(rs_lidar VERSION 0.3.5 LANGUAGES C CXX)

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
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)


# set(CMAKE_BUILD_TYPE Debug) # 默认Debug模式，可修改为Release

# 构建类型配置（默认Release）
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING 
        "Choose the build type: Debug Release" FORCE)
endif()


# 根据构建类型设置编译选项
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    # Debug模式：无优化 + 完整调试信息
    message(STATUS "Build Mode: Debug (-O0 + 完整调试信息)")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g")
    add_compile_options(-g -O0)
    add_link_options(-g)

elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    # Release模式：-O3极致优化，关闭调试信息
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -g0")
    message(STATUS "Build Mode: Release (默认，-O3 极致优化)")
endif()


# ==================== 编译选项开关 ====================
option(BUILD_PYTHON_PACKAGES "Build Python packages" ON)
option(COMPILE_DEMO "Compile demo programs" ON)
option(USE_PCL_POINT_TYPE "Use PCL point cloud type instead of custom type" ON)
option(EXECUTIONLIB "Include execution in .cpp" OFF)
option(PRINT_PARAMETER "cout parameters of lidar to console" OFF)
option(PRINT_DEBUG "cout debug to console" OFF)
option(PRINT_MSG "cout brief message of every frames to console when 'getPointCloud' call" OFF)
option(RS_TIME_RECORD "cout time record to console" OFF)

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

if(RS_TIME_RECORD STREQUAL ON)
  message(STATUS "cout time record to console")
  add_definitions(-DRS_TIME_RECORD)
endif()


# ==================== 依赖库查找 ======================
find_package(rs_driver REQUIRED)
if(NOT rs_driver_FOUND)
  message(FATAL_ERROR "rs_driver not found. Please install it first.")
endif()

if(USE_PCL_POINT_TYPE)
  include_directories(${PCL_INCLUDE_DIRS})
  link_libraries(${PCL_LIBRARIES})
endif()


# ==================== 全局包含目录 ====================
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/rs_type/include  # rs_type头文件目录
)

# ==================== 目标编译 ========================

# 编译rs_viewer库（可视化库）
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/rs_viewer)

# 编译rs_reader库（核心激光雷达读取库）
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/rs_reader)

# ==================== Demo程序编译 ====================
if(COMPILE_DEMO STREQUAL ON)
  message(STATUS "启用Demo程序构建")
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/demo)
endif()


# ==================== Python绑定配置 ==================
if(BUILD_PYTHON_PACKAGES STREQUAL ON)
  message(STATUS "启用Python绑定构建")
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/rs_bind)
endif()