#pragma once
#ifndef RS_POINT_CLOUD_TYPE_H_
#define RS_POINT_CLOUD_TYPE_H_


// 条件编译：根据是否启用PCL点类型，选择不同的点云数据结构实现
// 启用PCL时（定义USE_PCL_POINT_TYPE）：使用PCL库的点类型及扩展
// 禁用PCL时：使用自定义轻量级点类型（无第三方依赖）
#ifdef USE_PCL_POINT_TYPE

// PCL库依赖头文件
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <memory>      // 用于std::shared_ptr智能指针
#include <string>
#include <cstdint>     // 用于std::uint16_t等类型


// 命名空间：封装RoboSense点云相关类型
namespace robosense::type {

  // 类型别名：使用PCL库的PointXYZI作为基础点类型（包含x,y,z坐标及反射强度）
  using PointXYZI = pcl::PointXYZI;

  /**
   * @brief 扩展点类型：包含环号(ring)和时间戳(timestamp)的XYZI点
   * @note 用于需要区分激光雷达扫描环数和每个点时间戳的场景
   */
  struct PointXYZIRT
  {
    PCL_ADD_POINT4D;                  // PCL宏：添加x, y, z坐标及对齐填充
    float intensity;                  // 反射强度
    std::uint16_t ring;               // 激光环号（区分不同激光发射源）
    double timestamp;                 // 点的采集时间戳（秒）

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // Eigen库内存对齐宏
  } EIGEN_ALIGN16;                    // 16字节对齐，优化Eigen运算

  /**
   * @brief 点云容器类：继承PCL点云类并扩展元信息
   * @tparam T_Point 点类型（如PointXYZI、PointXYZIRT）
   * @note 在PCL点云基础上增加时间戳、帧序号、坐标系ID等元数据
   */
  template <typename T_Point>
  class PointCloudT : public pcl::PointCloud<T_Point>
  {
  public:
    using PointT = T_Point;                       // 点类型别名
    using VectorT = typename pcl::PointCloud<T_Point>::VectorType;  // 点容器类型别名

    double timestamp = 0.0;                       // 点云帧时间戳（秒）
    uint32_t seq = 0;                             // 帧序号（自增，用于时序同步）
    std::string frame_id = "";                    // 坐标系ID（用于多传感器坐标转换）
  };

  // 点类型选择：默认使用PointXYZI（可根据需求切换为PointXYZIRT）
  using PointT = PointXYZI;

  // 点云消息类型别名：基于选定的点类型
  using PointCloudMsg = PointCloudT<PointT>;
  using PointCloudMsgPtr = std::shared_ptr<PointCloudMsg>;  // 点云消息智能指针

}  // namespace robosense::type


// PCL宏：注册自定义点类型PointXYZIRT，使其能被PCL库识别和处理
POINT_CLOUD_REGISTER_POINT_STRUCT(
    robosense::type::PointXYZIRT,
    (float, x, x)              // X坐标
    (float, y, y)              // Y坐标
    (float, z, z)              // Z坐标
    (float, intensity, intensity)  // 反射强度
    (std::uint16_t, ring, ring)    // 激光环号
    (double, timestamp, timestamp)  // 时间戳
)

// 不启用PCL时：使用自定义轻量级点类型（无第三方依赖）
#else

#include <vector>
#include <string>
#include <cstdint>
#include <memory>


// 紧凑内存对齐：1字节对齐，减少内存占用（适用于资源受限场景）
#pragma pack(push, 1)

// 命名空间：封装RoboSense点云相关类型
namespace robosense::type {

  /**
   * @brief 轻量级点结构体（XYZI格式）
   * 包含三维坐标和反射强度，无第三方依赖
   */
  struct PointXYZI
  {
    float x;           ///< X轴坐标（单位：米）
    float y;           ///< Y轴坐标（单位：米）
    float z;           ///< Z轴坐标（单位：米）
    uint8_t intensity; ///< 反射强度（范围：0-255）
  };

}  // namespace robosense::type

// 恢复默认内存对齐方式
#pragma pack(pop)


// 命名空间：封装RoboSense点云相关类型（续）
namespace robosense::type {

  /**
   * @brief 轻量级点云容器类（模板）
   * 存储点云数据及元信息，支持自定义点类型，无第三方依赖
   * @tparam T_Point 点的类型（如PointXYZI）
   */
  template <typename T_Point>
  class PointCloudT
  {
  public:
    using PointT = T_Point;          ///< 点类型别名
    using VectorT = std::vector<PointT>;  ///< 点容器类型（std::vector）

    uint32_t height = 0;           ///< 点云高度（如扫描线数，0表示无序点云）
    uint32_t width = 0;            ///< 点云宽度（如每线点数，无序点云时为总点数）
    bool is_dense = false;         ///< 是否为密集点云（true表示无NaN/Inf等无效点）
    double timestamp = 0.0;        ///< 点云帧时间戳（单位：秒）
    uint32_t seq = 0;              ///< 帧序号（自增，用于时序同步）
    std::string frame_id = "";     ///< 坐标系ID（用于多传感器坐标转换）

    VectorT points;  ///< 点数据容器（存储所有点的集合）
  };

  // 点类型选择：默认使用自定义轻量级PointXYZI
  using PointT = PointXYZI;

  // 点云消息类型别名：基于自定义点类型
  using PointCloudMsg = PointCloudT<PointT>;         ///< 点云消息类型
  using PointCloudMsgPtr = std::shared_ptr<PointCloudMsg>;  ///< 点云消息智能指针（便于内存管理）

}  // namespace robosense::type

#endif

#endif  // RS_POINT_CLOUD_TYPE_H_