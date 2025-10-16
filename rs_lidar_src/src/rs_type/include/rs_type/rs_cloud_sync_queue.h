#ifndef _RS_CLOUD_SYNC_QUEUE_H_
#define _RS_CLOUD_SYNC_QUEUE_H_
#pragma once

#include "rs_point_cloud_type.h"
#include <rs_driver/utility/sync_queue.hpp>

namespace robosense::type {

/**
 * @brief 点云同步队列类型定义
 * 封装robosense::lidar::SyncQueue，用于线程安全的点云消息（PointCloudMsgPtr）传递
 */
using CloudSyncQueue = robosense::lidar::SyncQueue<PointCloudMsgPtr>;

}  // namespace robosense::type
#endif  // _RS_CLOUD_SYNC_QUEUE_H_
