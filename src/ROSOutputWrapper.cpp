/**
 * ROS driver for DM-VIO (converted to ROS2 Humble)
 * Original code by Lukas von Stumberg (http://vision.in.tum.de/dm-vio).
 *
 * DM-VIO is free software distributed under the terms of the GNU GPL.
 */

#include "ROSOutputWrapper.h"

#include <GTSAMIntegration/PoseTransformationIMU.h>
#include "std_msgs/msg/int32.hpp"
#include "dmvio_ros2/msg/dmvio_pose_msg.hpp"
#include "util/FrameShell.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

// For numeric limits and assertions.
#include <limits>
#include <cassert>

using namespace dmvio;

// Helper function to set a geometry_msgs::msg::Pose from a Sophus::SE3d pose.
void setMsgFromSE3(geometry_msgs::msg::Pose &poseMsg, const Sophus::SE3d &pose)
{
    poseMsg.position.x = pose.translation()[0];
    poseMsg.position.y = pose.translation()[1];
    poseMsg.position.z = pose.translation()[2];
    poseMsg.orientation.x = pose.so3().unit_quaternion().x();
    poseMsg.orientation.y = pose.so3().unit_quaternion().y();
    poseMsg.orientation.z = pose.so3().unit_quaternion().z();
    poseMsg.orientation.w = pose.so3().unit_quaternion().w();
}

ROSOutputWrapper::ROSOutputWrapper()
  : rclcpp::Node("dmvio")  // Initialize the node with the name "dmvio"
{
    // Create publishers with a QoS history depth of 10.
    systemStatePublisher = this->create_publisher<std_msgs::msg::Int32>("system_status", rclcpp::QoS(10));
    dmvioPosePublisher = this->create_publisher<dmvio_ros2::msg::DMVIOPoseMsg>("frame_tracked", rclcpp::QoS(10));
    unscaledPosePublisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("unscaled_pose", rclcpp::QoS(10));
    metricPosePublisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("metric_pose", rclcpp::QoS(10));
}

void ROSOutputWrapper::publishTransformDSOToIMU(const TransformDSOToIMU &transformDSOToIMUPassed)
{
    std::unique_lock<std::mutex> lk(mutex);
    transformDSOToIMU = std::make_unique<dmvio::TransformDSOToIMU>(
        transformDSOToIMUPassed,
        std::make_shared<bool>(false),
        std::make_shared<bool>(false),
        std::make_shared<bool>(false)
    );

    // Update availability of a valid scale.
    scaleAvailable = (lastSystemStatus == SystemStatus::VISUAL_INERTIAL);

    // Optionally you could publish the new scale (and gravity direction) immediately.
}

void ROSOutputWrapper::publishSystemStatus(dmvio::SystemStatus systemStatus)
{
    std_msgs::msg::Int32 msg;
    msg.data = static_cast<int>(systemStatus);
    systemStatePublisher->publish(msg);
    lastSystemStatus = systemStatus;
}

void ROSOutputWrapper::publishCamPose(dso::FrameShell *frame, dso::CalibHessian *HCalib)
{
    dmvio_ros2::msg::DMVIOPoseMsg msg;

    // Set header timestamp from frame->timestamp.
    // Assuming frame->timestamp is in seconds, use from_seconds.
    msg.header.stamp = rclcpp::Time(frame->timestamp);
    msg.header.frame_id = "world";

    // Get the camera-to-world transformation.
    auto &camToWorld = frame->camToWorld;

    // Set the pose in the custom message.
    setMsgFromSE3(msg.pose, camToWorld);

    // Also publish the unscaled pose on its own (e.g. for visualization in RViz).
    geometry_msgs::msg::PoseStamped unscaledMsg;
    unscaledMsg.header = msg.header;
    unscaledMsg.pose = msg.pose;
    unscaledPosePublisher->publish(unscaledMsg);

    {
        std::unique_lock<std::mutex> lk(mutex);
        if (transformDSOToIMU && scaleAvailable)
        {
            msg.scale = transformDSOToIMU->getScale();

            // Publish the scaled (metric) pose.
            geometry_msgs::msg::PoseStamped scaledMsg;
            scaledMsg.header = msg.header;
            // Transform to metric (IMU-to-world). Note that the inverse is needed since transformDSOToIMU expects worldToCam.
            Sophus::SE3d imuToWorld(transformDSOToIMU->transformPose(camToWorld.inverse().matrix()));
            setMsgFromSE3(scaledMsg.pose, imuToWorld);
            metricPosePublisher->publish(scaledMsg);
        }
        else
        {
            msg.scale = std::numeric_limits<double>::quiet_NaN();
            if (transformDSOToIMU)
            {
                assert(transformDSOToIMU->getScale() == 1.0);
            }
        }

        if (transformDSOToIMU)
        {
            // Get the rotation from metric to DSO (gravity direction).
            Sophus::SO3d gravityDirection = transformDSOToIMU->getR_dsoW_metricW();
            msg.rotation_metric_to_dso.x = gravityDirection.unit_quaternion().x();
            msg.rotation_metric_to_dso.y = gravityDirection.unit_quaternion().y();
            msg.rotation_metric_to_dso.z = gravityDirection.unit_quaternion().z();
            msg.rotation_metric_to_dso.w = gravityDirection.unit_quaternion().w();
            setMsgFromSE3(msg.imu_to_cam, transformDSOToIMU->getT_cam_imu());
        }
    }

    dmvioPosePublisher->publish(msg);
}
