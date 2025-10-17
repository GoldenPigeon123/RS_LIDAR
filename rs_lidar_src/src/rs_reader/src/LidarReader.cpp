#include "rs_reader/LidarReader.h"
#include <rs_driver/driver/driver_param.hpp>
#include <rs_driver/common/rs_log.hpp>
#include <filesystem>
#include <stdexcept>
#include <chrono>
#include <thread>

#ifdef DEXECUTIONLIB
#include <execution>  // 并行执行支持（可选编译宏）
#endif

/**
 * @brief 同步队列初始大小
 * @note 预分配100个点云缓冲区，减少运行时动态内存分配开销
 */
#define SYNC_QUEUE_INIT_SIZE 100

namespace robosense::reader {
namespace fs = std::filesystem;  // 文件系统操作命名空间别名

/**
 * @brief 构造函数：初始化激光雷达读取器基本参数
 * @param lidar_type_str 激光雷达型号字符串（如"RSE1"）
 * @param input_type_str 输入模式字符串（"ONLINE_LIDAR"或"PCAP_FILE"）
 * @param msop_port MSOP数据端口号
 * @param difop_port DIFOP数据端口号
 * @note 将字符串类型的雷达型号和输入模式转换为枚举值存储
 */
LidarReader::LidarReader(
    const std::string& lidar_type_str,
    const std::string& input_type_str,
    uint16_t msop_port,
    uint16_t difop_port
) : lidar_type_(robosense::lidar::strToLidarType(lidar_type_str)),
    input_type_(robosense::lidar::strToInputType(input_type_str)),
    msop_port_(msop_port),
    difop_port_(difop_port),
    pcap_file_path_(""),
    is_running_(false), 
    temperature_(-999.0f) {  // 初始温度设为无效值
        RS_DEBUG << "LidarReader::Constructor - lidar_type=" << robosense::lidar::lidarTypeToStr(lidar_type_) 
                << ", input_type=" << robosense::lidar::inputTypeToStr(input_type_) << RS_REND;
};

/**
 * @brief 析构函数：停止读取器并释放资源
 */
LidarReader::~LidarReader() {
  RS_INFO << "LidarReader::Destructor - Stopping reader..." << RS_REND;
}

/**
 * @brief 设置PCAP文件路径（仅离线模式有效）
 * @param file_path PCAP文件路径
 * @note 若路径不存在，输出警告日志但不抛出异常（允许后续重新设置）
 */
void LidarReader::set_pcap_path(const std::string& file_path) {
    std::lock_guard<std::mutex> lock(mtx_);  // 线程安全保护
    pcap_file_path_ = file_path;
    
    // 检查PCAP文件是否存在（仅在离线模式下）
    if (input_type_ == robosense::lidar::InputType::PCAP_FILE && !fs::exists(pcap_file_path_)) {
        RS_WARNING << "PCAP file does not exist: " << pcap_file_path_ << RS_REND;
    }
}

/**
 * @brief 初始化激光雷达驱动
 * @return 初始化成功返回true，失败返回false
 * @details 配置驱动参数、注册回调函数、初始化点云缓冲区队列
 */
bool LidarReader::init() {
    RS_INFO << "LidarReader::init - Starting initialization" << RS_REND;

    // 配置驱动核心参数
    driver_param_.lidar_type = lidar_type_;
    driver_param_.input_type = input_type_;
    driver_param_.input_param.msop_port = msop_port_;
    driver_param_.input_param.difop_port = difop_port_;

    // 初始化PCAP相关配置（仅离线模式）
    if (driver_param_.input_type == robosense::lidar::InputType::PCAP_FILE) {
        // 检查PCAP文件路径有效性
        if (pcap_file_path_.empty() || !fs::exists(pcap_file_path_)) {
            RS_ERROR << "Invalid PCAP file path: " << pcap_file_path_ << RS_REND;
            return false;
        }
        driver_param_.input_param.pcap_path = pcap_file_path_;
        driver_param_.input_param.pcap_rate = 1.0;  // 回放速率（1.0表示原速）
        driver_param_.input_param.pcap_repeat = false;  // 不重复回放
    }

    // 注册驱动回调函数
    // 1. 点云缓冲区获取回调：驱动从空闲队列获取缓冲区
    // 2. 点云返回回调：驱动将处理完的点云放入已填充队列
    driver_.regPointCloudCallback(
        std::bind(&LidarReader::driverGetPointCloudFromCallerCallback, this),
        std::bind(&LidarReader::driverReturnPointCloudToCallerCallback, this, std::placeholders::_1)
    );
    // 注册异常回调：处理驱动运行中的错误/警告
    driver_.regExceptionCallback(
        std::bind(&LidarReader::exceptionCallback, this, std::placeholders::_1)
    );

    // 初始化点云缓冲区队列（预分配内存，减少动态分配）
    for (int i = 0; i < SYNC_QUEUE_INIT_SIZE; ++i) {
        free_cloud_queue_.push(std::make_shared<PointCloudMsg>());
    }

    // 若未注册自定义过滤函数，默认使用欧式距离过滤
    if(filter_func_ == nullptr) {
        filter_func_ = std::bind(
            &LidarReader::euclidianDistanceFilter, 
            this, 
            std::placeholders::_1, 
            std::placeholders::_2, 
            std::placeholders::_3
        );
    }
    
#ifdef PRINT_PARAMETER
    driver_param_.print();  // 打印驱动参数（调试用）
#endif

    // 初始化驱动核心
    if (!driver_.init(driver_param_)) {
        RS_ERROR << "Lidar driver init failed" << RS_REND;
        return false;
    }

    RS_INFO << "LidarReader::init - Succeeded" << RS_REND;
    return true;
}

/**
 * @brief 启动数据读取线程
 * @return 启动成功返回true，失败返回false
 * @note 线程安全，确保驱动仅启动一次
 */
bool LidarReader::start() {
    std::lock_guard<std::mutex> lock(mtx_);  // 线程安全保护
    if (is_running_) {
        RS_WARNING << "Reader is already running" << RS_REND;
        return true;
    }

    // 启动驱动内部线程（开始接收/解析数据）
    if (!driver_.start()) {
        RS_ERROR << "Lidar driver start failed" << RS_REND;
        return false;
    }

    is_running_ = true;  // 更新运行状态标志
    RS_INFO << "LidarReader::start - Succeeded" << RS_REND;
    return true;
}

/**
 * @brief 停止数据读取线程
 * @note 线程安全，确保驱动仅停止一次
 */
void LidarReader::stop() {
    std::lock_guard<std::mutex> lock(mtx_);  // 线程安全保护
    if (!is_running_) {
        RS_WARNING << "Reader is not running" << RS_REND;
        return;
    }

    // 停止驱动并更新状态
    is_running_ = false;
    driver_.stop();

    RS_INFO << "LidarDriver::stop - Stopped" << RS_REND;
}

/**
 * @brief 驱动回调：从空闲队列获取点云缓冲区
 * @return 空闲的点云消息指针（队列空时创建新对象）
 * @note 供驱动内部调用，用于获取存储点云数据的缓冲区
 */
PointCloudMsgPtr LidarReader::driverGetPointCloudFromCallerCallback() {
    PointCloudMsgPtr msg = free_cloud_queue_.pop();
    return msg ? msg : std::make_shared<PointCloudMsg>();  // 队列空则新建
}

/**
 * @brief 驱动回调：处理并存储解析完成的点云
 * @param msg 驱动解析完成的点云消息指针
 * @details 1. 过滤无效点（NaN/Inf）和需过滤的点（通过过滤函数）
 *          2. 将处理后的点云放入已填充队列，供上层获取
 */
void LidarReader::driverReturnPointCloudToCallerCallback(PointCloudMsgPtr msg) {
#ifdef RS_TIME_RECORD
    auto start_time = std::chrono::high_resolution_clock::now();  // 计时开始
#endif

    // 过滤无效点和需过滤的点
    // 若启用DEXECUTIONLIB，使用并行算法加速过滤
    auto it = std::remove_if(
#ifdef DEXECUTIONLIB
        std::execution::par,  // 并行执行（多线程加速）
#endif
        msg->points.begin(), msg->points.end(),
        [this](const PointT& p) {
            // 过滤条件：坐标非有限值 或 过滤函数返回true
            return !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || filter_func_(p.x, p.y, p.z);
        });
    msg->points.erase(it, msg->points.end());  // 移除过滤后的点

    // 将处理后的点云放入已填充队列
    stuffed_cloud_queue_.push(msg);
    
#ifdef RS_TIME_RECORD
    auto end_time = std::chrono::high_resolution_clock::now();  // 计时结束
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
    RS_DEBUG << "driverReturnPointCloudToCallerCallback: " << duration << "us" << RS_REND;
#endif
    
#ifdef PRINT_MSG
    RS_MSG << " PointCloudToCallerCallback :seq-" << msg->seq << " size:"<<msg->points.size() << RS_REND;
#endif

}

/**
 * @brief 异常回调：处理驱动运行中的错误/警告
 * @param err 错误信息对象
 * @note 线程安全，根据错误类型输出对应日志
 */
void LidarReader::exceptionCallback(const robosense::lidar::Error& err) {
    std::lock_guard<std::mutex> lock(mtx_);  // 线程安全保护
    is_exception_callback_occurred_ = true;  // 标记异常发生

    // 根据错误类型输出日志
    switch(err.error_code_type) {
        case robosense::lidar::ErrCodeType::ERROR_CODE: 
            RS_ERROR << err.toString() << RS_REND;
            return ;
        case robosense::lidar::ErrCodeType::WARNING_CODE:
            RS_WARNING << err.toString() << RS_REND;
            return ;
        case robosense::lidar::ErrCodeType::INFO_CODE:
            RS_INFO << err.toString() << RS_REND;
            return ;
    }
    
}

/**
 * @brief 向用户提供的接口：获取点云数据
 * @param usec 超时时间（微秒）
 * @return 点云消息指针（超时或异常时返回nullptr）
 */
PointCloudMsgPtr LidarReader::getPointCloud(unsigned int usec) {
    
    // 检查运行状态：未运行或发生异常时返回nullptr
    if (!is_running_ || is_exception_callback_occurred_) {
        RS_WARNING << "getPointCloud failed: reader not running or exception callback occurred" << RS_REND;
        return nullptr;
    }

    // 从已填充队列中等待获取点云（带超时）
    PointCloudMsgPtr msg = stuffed_cloud_queue_.popWait(usec);
    if (!msg) {
        return nullptr;  // 超时返回nullptr
    }
    
    return msg;
}

/**
 * @brief 向用户提供的接口：释放点云缓冲区（放回空闲队列）
 * @param msg 待释放的点云消息指针
 * @note 清空点数据但保留缓冲区，实现内存复用
 */
void LidarReader::freePointCloud(const PointCloudMsgPtr& msg) {
    if (msg) {
        msg->points.clear();  // 清空点数据
        free_cloud_queue_.push(msg);  // 放回空闲队列
    }
}

/**
 * @brief 获取激光雷达温度
 * @return 温度值（摄氏度）
 * @throws std::runtime_error 温度获取失败时抛出异常
 */
float LidarReader::getTemperature() {
    if (driver_.getTemperature(temperature_)) {
        return temperature_;
    } else {
        throw std::runtime_error("Temperature not available");
    }
}

/**
 * @brief 打印驱动参数配置
 */
void LidarReader::printDriverParam() {
    driver_param_.print();
};

/**
 * @brief 打印设备信息（型号、序列号等）
 */
void LidarReader::printDeviceInfo() {
    driver_.getDeviceInfo(device_info_);
    device_info_.print();
};

/**
 * @brief 打印设备状态（运行状态、错误码等）
 */
void LidarReader::printDeviceStatus() {
    driver_.getDeviceStatus(device_status_);
    device_status_.print();
};

}  // namespace robosense::reader