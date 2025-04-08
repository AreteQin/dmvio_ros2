/**
 * ROS driver for DM-VIO (converted to ROS2 Humble)
 * Original code by Lukas von Stumberg (http://vision.in.tum.de/dm-vio).
 *
 * Copyright (c) 2022 Lukas von Stumberg
 *
 * DM-VIO is free software distributed under the terms of the GNU GPL.
 *
 * If you use this code, please cite the respective publications as listed on
 * the website.
 */

#ifndef DMVIO_ROS_ROSOUTPUTWRAPPER_HPP_
#define DMVIO_ROS_ROSOUTPUTWRAPPER_HPP_

#include <IOWrapper/Output3DWrapper.h>
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <atomic>
#include <memory>

// ROS2 message includes
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
// Note: Ensure that the DM-VIO custom message is available in your workspace.
#include "dmvio_ros2/msg/dmvio_pose_msg.hpp"

namespace dmvio
{

// We publish 3 topics by default:
//  - "frame_tracked": dmvio_ros2::msg::DMVIOPoseMsg
//  - "unscaled_pose": geometry_msgs::msg::PoseStamped
//  - "metric_pose": geometry_msgs::msg::PoseStamped
// In addition we publish "system_status" (std_msgs::msg::Int32).
class ROSOutputWrapper : public dso::IOWrap::Output3DWrapper, public rclcpp::Node
{
public:
    ROSOutputWrapper();

    /**
     * Called once after each keyframe is optimized and passes the new transformation
     * from DSO frame (worldToCam in DSO scale) to metric frame (imuToWorld in metric scale).
     *
     * The passed transform can be used (via transformPose) to transform poses between the frames.
     * Note that the object should not be used any more after the method returns.
     */
    virtual void publishTransformDSOToIMU(const dmvio::TransformDSOToIMU& transformDSOToIMU) override;

    /**
     * Called every time the system status changes.
     */
    virtual void publishSystemStatus(dmvio::SystemStatus systemStatus) override;

    /**
     * Called for each tracked frame, with the real-time low-delay frame pose.
     */
    virtual void publishCamPose(dso::FrameShell* frame, dso::CalibHessian* HCalib) override;

private:
    // Publishers using ROS2 interfaces.
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr systemStatePublisher;
    rclcpp::Publisher<dmvio_ros2::msg::DMVIOPoseMsg>::SharedPtr dmvioPosePublisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr unscaledPosePublisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr metricPosePublisher;

    // Protects transformDSOToIMU.
    std::mutex mutex;

    std::unique_ptr<dmvio::TransformDSOToIMU> transformDSOToIMU;
    bool scaleAvailable = false;
    std::atomic<dmvio::SystemStatus> lastSystemStatus;
};

}  // namespace dmvio

#endif // DMVIO_ROS_ROSOUTPUTWRAPPER_HPP_
