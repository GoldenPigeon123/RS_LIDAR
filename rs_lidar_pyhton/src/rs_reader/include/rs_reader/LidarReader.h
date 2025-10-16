#ifndef RS_READER_LIDAR_READER_H
#define RS_READER_LIDAR_READER_H

#include <string>
#include <memory>
#include <atomic>
#include <mutex>
#include <rs_driver/api/lidar_driver.hpp>
#include <rs_driver/msg/packet.hpp>
#include <rs_driver/driver/driver_param.hpp>
#include <rs_driver/common/rs_log.hpp>
#include <rs_driver/common/error_code.hpp>

#include "rs_type/rs_point_cloud_type.h"
#include "rs_type/rs_cloud_sync_queue.h"

// 类型别名：简化点云相关类型的使用
using robosense::type::PointT;               ///< 点云点类型（默认XYZI）
using robosense::type::PointCloudMsg;        ///< 点云消息类型（包含点数据及元信息）
using robosense::type::PointCloudMsgPtr;     ///< 点云消息智能指针（便于内存管理）
using robosense::type::CloudSyncQueue;       ///< 线程安全的点云队列（用于缓冲区管理）

/**
 * @brief 默认欧式距离过滤阈值（单位：米）
 * @note 用于过滤超出该距离的点（x² + y² + z² > 阈值² 时被过滤）
 */
#define DEFAULT_EUCLID_DISTANCE_EPSILON (5.0)

namespace robosense::reader {

/**
 * @brief 激光雷达数据读取器类
 * @details 封装RoboSense激光雷达驱动核心逻辑，提供统一接口支持两种工作模式：
 *          1. 在线模式（ONLINE_LIDAR）：实时接收激光雷达的MSOP/DIFOP数据包并解析为点云
 *          2. 离线模式（PCAP_FILE）：从PCAP文件中解析数据包并生成点云
 *          内部通过双缓冲队列实现点云内存复用，支持自定义点云过滤规则，提供设备信息查询接口
 */
class LidarReader {
public:
  /**
   * @brief 构造函数：初始化激光雷达基本参数
   * @param lidar_type_str 激光雷达型号字符串（如"RSE1"、"RS16"等，默认"RSE1"）
   * @param input_type_str 输入模式字符串（"ONLINE_LIDAR"表示在线模式，"PCAP_FILE"表示离线PCAP模式，默认在线）
   * @param msop_port MSOP数据包接收端口号（激光雷达主数据端口，默认6699）
   * @param difop_port DIFOP数据包接收端口号（激光雷达配置数据端口，默认7788）
   */
  LidarReader(
    const std::string& lidar_type_str = "RSE1",
    const std::string& input_type_str = "ONLINE_LIDAR",
    uint16_t msop_port = 6699,
    uint16_t difop_port = 7788
  );
  
  /**
   * @brief 析构函数：停止驱动并释放资源
   */
  ~LidarReader();

  // 禁用拷贝和移动构造/赋值：避免多实例导致的资源竞争（驱动实例应唯一）
  LidarReader(const LidarReader&) = delete;
  LidarReader& operator=(const LidarReader&) = delete;
  LidarReader(LidarReader&&) = delete;
  LidarReader& operator=(LidarReader&&) = delete;

  /**
   * @brief 设置PCAP文件路径（仅在PCAP_FILE模式下有效）
   * @param file_path 待解析的PCAP文件完整路径（如"/data/radar.pcap"）
   */
  void set_pcap_path(const std::string& file_path);

  /**
   * @brief 设置欧式距离过滤阈值
   * @param distance 过滤阈值（单位：米），超出该距离的点将被过滤
   */
  void set_distance_epsilon(float distance){
    distance_filter_=distance; 
  }

  /**
   * @brief 注册自定义点云过滤函数
   * @param func 过滤函数，参数为点的x/y/z坐标，返回true表示该点应被过滤，false表示保留
   * @note 自定义函数优先级高于默认的欧式距离过滤
   */
  void regFilterFunction(const std::function<bool(float,float,float)>& func){
    filter_func_=func;
  }

  /**
   * @brief 初始化激光雷达驱动
   * @details 配置驱动参数、注册回调函数，完成驱动初始化
   * @return 初始化成功返回true，失败返回false
   */
  bool init();

  /**
   * @brief 启动数据读取线程
   * @details 启动驱动内部线程，开始接收/解析数据并生成点云
   * @return 启动成功返回true，失败返回false
   */
  bool start();

  /**
   * @brief 停止数据读取线程
   * @details 停止驱动内部线程，释放相关资源
   */
  void stop();

  /**
   * @brief 检查驱动是否正在运行
   * @return 驱动运行中返回true，已停止返回false
   */
  bool isDriverRunning() const { return is_running_; }
  
  /**
   * @brief 获取处理后的点云数据
   * @param usec 超时时间（单位：微秒，默认1秒），超时未获取到点云则返回nullptr
   * @return 成功获取到点云时返回PointCloudMsgPtr，超时或失败返回nullptr
   */
  PointCloudMsgPtr getPointCloud(unsigned int usec = 1000000);

  /**
   * @brief 释放点云消息（归还至空闲队列实现内存复用）
   * @param msg 待释放的点云消息指针（需为通过getPointCloud获取的指针）
   */
  void freePointCloud(const PointCloudMsgPtr& msg);
  
  /**
   * @brief 获取激光雷达当前温度
   * @return 温度值（单位：摄氏度）
   */
  float getTemperature();

  /**
   * @brief 打印驱动当前配置参数
   * @details 包括端口号、雷达型号、工作模式等配置信息
   */
  void printDriverParam();

  /**
   * @brief 打印激光雷达设备信息
   * @details 包括设备型号、序列号、固件版本等硬件信息
   */
  void printDeviceInfo();

  /**
   * @brief 打印激光雷达当前运行状态
   * @details 包括运行状态码、错误信息（如有）等
   */
  void printDeviceStatus();

private:
  robosense::lidar::LidarType lidar_type_;  ///< 激光雷达类型（枚举值，由型号字符串转换而来）
  robosense::lidar::InputType input_type_;  ///< 输入模式（枚举值，ONLINE_LIDAR或PCAP_FILE）
  uint16_t msop_port_;                      ///< MSOP数据端口号（在线模式使用）
  uint16_t difop_port_;                     ///< DIFOP数据端口号（在线模式使用）
  std::string pcap_file_path_;              ///< PCAP文件路径（仅离线模式有效）
  
  robosense::lidar::RSDriverParam driver_param_;  ///< 激光雷达驱动配置参数（包含端口、类型等）
  robosense::lidar::DeviceInfo device_info_;      ///< 激光雷达设备信息（型号、固件版本等）
  robosense::lidar::DeviceStatus device_status_;  ///< 激光雷达设备状态（运行状态、错误码等）
  
  using LidarDriverType = robosense::lidar::LidarDriver<PointCloudMsg>;
  LidarDriverType driver_;  ///< 激光雷达驱动实例（核心驱动对象）
  
  std::atomic<bool> is_running_{false};  ///< 驱动运行状态标志（原子变量，线程安全）
  std::atomic<bool> is_exception_callback_occurred_{false};  ///< 异常发生标志（原子变量）
  std::function<bool(float,float,float)> filter_func_;  ///< 自定义点云过滤函数（用户注册）
  std::atomic<float> distance_filter_=DEFAULT_EUCLID_DISTANCE_EPSILON;  ///< 欧式距离过滤阈值（原子变量）
  float temperature_{0.0f};              ///< 缓存的设备温度值
  
  CloudSyncQueue free_cloud_queue_;      ///< 空闲点云队列（用于缓存可复用的点云缓冲区）
  CloudSyncQueue stuffed_cloud_queue_;   ///< 已填充点云队列（存储解析完成的点云，供上层获取）
  mutable std::mutex mtx_;               ///< 互斥锁（用于保护设备信息等共享资源的线程安全访问）

  /**
   * @brief 欧式距离过滤实现
   * @param x 点的X坐标（米）
   * @param y 点的Y坐标（米）
   * @param z 点的Z坐标（米）
   * @return 点到原点的距离平方大于阈值平方时返回true（过滤该点），否则返回false（保留）
   */
  bool euclidianDistanceFilter(float x,float y,float z){
    return x*x+y*y+z*z>distance_filter_*distance_filter_;
  };

  /**
   * @brief 驱动回调：从调用者获取点云缓冲区
   * @details 驱动解析数据前调用，从空闲队列中获取一个点云缓冲区供驱动填充数据
   * @return 空闲的点云消息指针（若队列空则创建新对象）
   */
  PointCloudMsgPtr driverGetPointCloudFromCallerCallback();

  /**
   * @brief 驱动回调：将处理完的点云返回给调用者
   * @details 驱动解析完成后调用，将处理后的点云放入已填充队列，供上层通过getPointCloud获取
   * @param msg 已填充完成的点云消息指针
   */
  void driverReturnPointCloudToCallerCallback(PointCloudMsgPtr msg);

  /**
   * @brief 异常回调：处理驱动运行中产生的错误
   * @details 当驱动发生错误（如端口占用、PCAP文件不存在等）时调用，记录错误信息
   * @param err 错误信息对象（包含错误码和描述）
   */
  void exceptionCallback(const robosense::lidar::Error& err);
};

}  // namespace robosense::reader

#endif  // RS_READER_LIDAR_READER_H