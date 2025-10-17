#pragma once
#ifndef RS_POINT_CLOUD_TYPE_H_
#define RS_POINT_CLOUD_TYPE_H_


// 条件编译：根据是否启用PCL点类型，选择不同的点云数据结构实现
#ifdef USE_PCL_POINT_TYPE

// PCL库依赖头文件
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <memory>
#include <string>
#include <cstdint>
#include <utility>  // 用于std::move


// 命名空间：封装RoboSense点云相关类型
namespace robosense::type {

  // 类型别名：使用PCL库的PointXYZI作为基础点类型
  using PointXYZI = pcl::PointXYZI;

  /**
   * @brief 扩展点类型：包含环号(ring)和时间戳(timestamp)的XYZI点
   */
  struct PointXYZIRT
  {
    PCL_ADD_POINT4D;                  // PCL宏：添加x,y,z坐标及对齐填充
    float intensity;                  // 反射强度
    std::uint16_t ring;               // 激光环号
    double timestamp;                 // 点的采集时间戳（秒）

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // Eigen库内存对齐宏

    // 复制/移动语义（默认生成，基础类型+PCL宏成员无需手动实现）
    PointXYZIRT() = default;
    PointXYZIRT(const PointXYZIRT&) = default;
    PointXYZIRT& operator=(const PointXYZIRT&) = default;
    PointXYZIRT(PointXYZIRT&&) noexcept = default;
    PointXYZIRT& operator=(PointXYZIRT&&) noexcept = default;
  };

  /**
   * @brief 点云容器类：继承PCL点云类并扩展元信息
   * @tparam T_Point 点类型（如PointXYZI、PointXYZIRT）
   */
  template <typename T_Point>
  class PointCloudT : public pcl::PointCloud<T_Point>
  {
  public:
    // 关键：定义基类别名，避免直接写pcl::PointCloud<T_Point>
    using Base = pcl::PointCloud<T_Point>;
    using PointT = T_Point;

    double timestamp = 0.0;           // 扩展：点云帧时间戳
    uint32_t seq = 0;                 // 扩展：帧序号
    std::string frame_id = "";        // 扩展：坐标系ID

    // -------------------------- 构造函数（必须包含默认构造） --------------------------
    // 默认构造：调用基类默认构造，初始化扩展成员
    PointCloudT() : Base() {}

    // 带点数量的构造：预分配内存
    explicit PointCloudT(size_t num_points) : Base(num_points) {}

    // -------------------------- 复制语义（调用基类+复制扩展成员） --------------------------
    PointCloudT(const PointCloudT& other)
      : Base(other),  // 调用PCL基类的复制构造
        timestamp(other.timestamp),
        seq(other.seq),
        frame_id(other.frame_id)
    {}

    PointCloudT& operator=(const PointCloudT& other) {
      if (this != &other) {
        Base::operator=(other);  // 调用基类复制赋值
        timestamp = other.timestamp;
        seq = other.seq;
        frame_id = other.frame_id;
      }
      return *this;
    }

    // -------------------------- 移动语义（调用基类+移动扩展成员） --------------------------
    PointCloudT(PointCloudT&& other) noexcept
      : Base(std::move(other)),  // 调用PCL基类的移动构造
        timestamp(std::move(other.timestamp)),
        seq(std::move(other.seq)),
        frame_id(std::move(other.frame_id))
    {}

    PointCloudT& operator=(PointCloudT&& other) noexcept {
      if (this != &other) {
        Base::operator=(std::move(other));  // 调用基类移动赋值
        timestamp = std::move(other.timestamp);
        seq = std::move(other.seq);
        frame_id = std::move(other.frame_id);
      }
      return *this;
    }
  };

  // 点类型选择：默认使用PointXYZI
  using PointT = PointXYZI;

  // 点云消息类型别名
  using PointCloudMsg = PointCloudT<PointT>;
  using PointCloudMsgPtr = std::shared_ptr<PointCloudMsg>;

}  // namespace robosense::type


// PCL宏：注册自定义点类型PointXYZIRT
POINT_CLOUD_REGISTER_POINT_STRUCT(
    robosense::type::PointXYZIRT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint16_t, ring, ring)
    (double, timestamp, timestamp)
)

// 不启用PCL时：使用自定义轻量级点类型（无第三方依赖）
#else

#include <vector>
#include <string>
#include <cstdint>
#include <memory>
#include <utility>  // 用于std::move


// 紧凑内存对齐：1字节对齐，减少内存占用
#pragma pack(push, 1)

namespace robosense::type {

  /**
   * @brief 轻量级点结构体（XYZI格式）
   */
  struct PointXYZI
  {
    float x;           ///< X轴坐标（米）
    float y;           ///< Y轴坐标（米）
    float z;           ///< Z轴坐标（米）
    uint8_t intensity; ///< 反射强度（0-255）

    // 复制/移动语义（默认生成，基础类型无需手动实现）
    PointXYZI() = default;
    PointXYZI(const PointXYZI&) = default;
    PointXYZI& operator=(const PointXYZI&) = default;
    PointXYZI(PointXYZI&&) noexcept = default;
    PointXYZI& operator=(PointXYZI&&) noexcept = default;
  };

}  // namespace robosense::type

#pragma pack(pop)


namespace robosense::type {

  /**
   * @brief 轻量级点云容器类（无继承，独立实现）
   * @tparam T_Point 点的类型（如PointXYZI）
   */
  template <typename T_Point>
  class PointCloudT
  {
  public:
    using PointT = T_Point;
    using VectorT = std::vector<PointT>;

    // 基础成员（模仿PCL结构，便于兼容）
    uint32_t height = 0;           ///< 点云高度（0=无序点云）
    uint32_t width = 0;            ///< 点云宽度（无序时=总点数）
    bool is_dense = false;         ///< 是否密集点云（无无效点）
    // 扩展成员
    double timestamp = 0.0;        ///< 点云帧时间戳（秒）
    uint32_t seq = 0;              ///< 帧序号
    std::string frame_id = "";     ///< 坐标系ID
    // 点数据容器
    VectorT points;

    // -------------------------- 构造函数（必须包含默认构造） --------------------------
    // 默认构造：初始化所有成员为默认值
    PointCloudT() = default;

    // 带点数量的构造：预分配点容器内存
    explicit PointCloudT(size_t num_points) : points(num_points) {}

    // -------------------------- 复制语义（深复制所有成员） --------------------------
    PointCloudT(const PointCloudT& other)
      : height(other.height),
        width(other.width),
        is_dense(other.is_dense),
        timestamp(other.timestamp),
        seq(other.seq),
        frame_id(other.frame_id),
        points(other.points)  // vector深复制（独立数据）
    {}

    PointCloudT& operator=(const PointCloudT& other) {
      if (this != &other) {
        height = other.height;
        width = other.width;
        is_dense = other.is_dense;
        timestamp = other.timestamp;
        seq = other.seq;
        frame_id = other.frame_id;
        points = other.points;  // 深复制点数据
      }
      return *this;
    }

    // -------------------------- 移动语义（转移资源，无复制） --------------------------
    PointCloudT(PointCloudT&& other) noexcept
      : height(other.height),
        width(other.width),
        is_dense(other.is_dense),
        timestamp(std::move(other.timestamp)),
        seq(std::move(other.seq)),
        frame_id(std::move(other.frame_id)),
        points(std::move(other.points))  // vector移动（转移内部数据）
    {
      // 重置源对象（确保源对象处于“空但有效”状态）
      other.height = 0;
      other.width = 0;
      other.is_dense = false;
      other.timestamp = 0.0;
      other.seq = 0;
      other.frame_id.clear();
      // vector移动后自动清空，无需手动处理
    }

    PointCloudT& operator=(PointCloudT&& other) noexcept {
      if (this != &other) {
        height = other.height;
        width = other.width;
        is_dense = other.is_dense;
        timestamp = std::move(other.timestamp);
        seq = std::move(other.seq);
        frame_id = std::move(other.frame_id);
        points = std::move(other.points);  // 转移点数据

        // 重置源对象
        other.height = 0;
        other.width = 0;
        other.is_dense = false;
        other.timestamp = 0.0;
        other.seq = 0;
        other.frame_id.clear();
      }
      return *this;
    }
  };

  // 点类型选择：默认使用自定义PointXYZI
  using PointT = PointXYZI;

  // 点云消息类型别名
  using PointCloudMsg = PointCloudT<PointT>;
  using PointCloudMsgPtr = std::shared_ptr<PointCloudMsg>;

}  // namespace robosense::type

#endif

#endif  // RS_POINT_CLOUD_TYPE_H_