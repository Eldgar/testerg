#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "attach_shelf/srv/move_to_frame.hpp"
#include <cmath>

class MoveToFrameServer : public rclcpp::Node {
public:
    MoveToFrameServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("move_to_frame_server", options) {
        // Declare the 'cmd_vel_topic' parameter with a default value
        this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");

        // Get the parameter value
        std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();

        // Create the velocity publisher using the parameter
        vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

        // Create the service
        service_ = this->create_service<attach_shelf::srv::MoveToFrame>(
            "/move_to_frame",
            std::bind(&MoveToFrameServer::handle_service, this, std::placeholders::_1, std::placeholders::_2));

        // Initialize tf2 buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Log node initialization
        RCLCPP_INFO(this->get_logger(), "MoveToFrameServer initialized with cmd_vel_topic: %s", cmd_vel_topic.c_str());
    }

private:
    rclcpp::Service<attach_shelf::srv::MoveToFrame>::SharedPtr service_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Tolerance values for precise movement
    const double POSITION_TOLERANCE = 0.04;  // meters
    const double POSITION_OFFSET = 0.35;
    const double ANGLE_TOLERANCE = 0.04;     // radians (~3 degrees)
    const double MAX_LINEAR_SPEED = 0.07; 
    const double MIN_LINEAR_SPEED = 0.015;
    const double MAX_ANGULAR_SPEED = 0.5;

    void handle_service(
        const std::shared_ptr<attach_shelf::srv::MoveToFrame::Request> request,
        std::shared_ptr<attach_shelf::srv::MoveToFrame::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Received request to move to frame: %s", request->target_frame.c_str());

        const std::string target_frame = request->target_frame;

        rclcpp::Rate rate(10);  // 10 Hz control loop
        bool success = false;

        // **Phase 1: Move to Adjusted Position**
        while (rclcpp::ok()) {
            // Get transform from `robot_base_footprint` to `target_frame`
            geometry_msgs::msg::TransformStamped transform;
            try {
                transform = tf_buffer_->lookupTransform("robot_base_footprint", target_frame, tf2::TimePointZero);
            } catch (const tf2::TransformException &ex) {
                RCLCPP_ERROR(this->get_logger(), "Could not transform to %s: %s", target_frame.c_str(), ex.what());
                response->success = false;
                response->message = "Transform lookup failed.";
                return;
            }

            // Extract position difference
            double dx = transform.transform.translation.x;
            double dy = transform.transform.translation.y;

            // Extract the target frame's orientation (yaw)
            tf2::Quaternion q(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w);

            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

            // Compute the adjusted target position using the target frame's orientation
            double adjusted_dx = dx - POSITION_OFFSET * std::cos(yaw);
            double adjusted_dy = dy - POSITION_OFFSET * std::sin(yaw);

            // Compute distance and angle to the adjusted target
            double adjusted_distance = std::hypot(adjusted_dx, adjusted_dy);
            double angle_to_adjusted_target = std::atan2(adjusted_dy, adjusted_dx);

            // Feedback for debugging
            RCLCPP_INFO(this->get_logger(), "Adjusted Distance: %.3f, Angle to Adjusted Target: %.3f", adjusted_distance, angle_to_adjusted_target);

            // Check if we are within position tolerance
            if (adjusted_distance < POSITION_TOLERANCE) {
                RCLCPP_INFO(this->get_logger(), "Reached adjusted target position for frame: %s", target_frame.c_str());
                success = true;
                break;  // Exit the position control loop
            }

            // Compute velocity commands
            geometry_msgs::msg::Twist cmd_vel;

            // Linear velocity (proportional control)
            cmd_vel.linear.x = std::min(MAX_LINEAR_SPEED, std::max(MIN_LINEAR_SPEED, adjusted_distance * 0.5));

            // Angular velocity (proportional control)
            cmd_vel.angular.z = std::max(-MAX_ANGULAR_SPEED, std::min(MAX_ANGULAR_SPEED, angle_to_adjusted_target * 2.0));

            // Publish velocity command
            vel_publisher_->publish(cmd_vel);

            // Sleep to maintain control loop rate
            rate.sleep();
        }

        // **Phase 2: Rotate to Match Target Orientation**
        if (success) {
            success = false;  // Reset success flag for the rotation phase
            while (rclcpp::ok()) {
                // Get the robot's current orientation relative to a fixed frame (e.g., 'map' or 'odom')
                geometry_msgs::msg::TransformStamped robot_transform;
                try {
                    robot_transform = tf_buffer_->lookupTransform("map", "robot_base_footprint", tf2::TimePointZero);
                } catch (const tf2::TransformException &ex) {
                    RCLCPP_ERROR(this->get_logger(), "Could not get robot's orientation: %s", ex.what());
                    response->success = false;
                    response->message = "Robot orientation lookup failed.";
                    return;
                }

                // Extract the robot's yaw angle
                tf2::Quaternion robot_q(
                    robot_transform.transform.rotation.x,
                    robot_transform.transform.rotation.y,
                    robot_transform.transform.rotation.z,
                    robot_transform.transform.rotation.w);
                double robot_roll, robot_pitch, robot_yaw;
                tf2::Matrix3x3(robot_q).getRPY(robot_roll, robot_pitch, robot_yaw);

                // Get the target frame's orientation relative to the same fixed frame
                geometry_msgs::msg::TransformStamped target_transform;
                try {
                    target_transform = tf_buffer_->lookupTransform("map", target_frame, tf2::TimePointZero);
                } catch (const tf2::TransformException &ex) {
                    RCLCPP_ERROR(this->get_logger(), "Could not get target frame's orientation: %s", ex.what());
                    response->success = false;
                    response->message = "Target frame orientation lookup failed.";
                    return;
                }

                // Extract the target frame's yaw angle
                tf2::Quaternion target_q(
                    target_transform.transform.rotation.x,
                    target_transform.transform.rotation.y,
                    target_transform.transform.rotation.z,
                    target_transform.transform.rotation.w);
                double target_roll, target_pitch, target_yaw;
                tf2::Matrix3x3(target_q).getRPY(target_roll, target_pitch, target_yaw);

                // Compute orientation error
                double orientation_error = target_yaw - robot_yaw;
                // Normalize the angle to the range [-π, π]
                orientation_error = std::atan2(std::sin(orientation_error), std::cos(orientation_error));

                // Feedback for debugging
                RCLCPP_INFO(this->get_logger(), "Orientation Error: %.3f", orientation_error);

                // Check if orientation error is within tolerance
                if (std::abs(orientation_error) < ANGLE_TOLERANCE) {
                    RCLCPP_INFO(this->get_logger(), "Robot orientation aligned with target frame.");
                    success = true;
                    break;  // Exit the rotation control loop
                }

                // Compute angular velocity command
                geometry_msgs::msg::Twist cmd_vel;
                cmd_vel.angular.z = std::max(-MAX_ANGULAR_SPEED, std::min(MAX_ANGULAR_SPEED, orientation_error));

                // Publish velocity command
                vel_publisher_->publish(cmd_vel);

                // Sleep to maintain control loop rate
                rate.sleep();
            }
        }

        // Stop the robot
        geometry_msgs::msg::Twist stop_cmd;
        vel_publisher_->publish(stop_cmd);

        // Respond with success or failure
        response->success = success;
        response->message = success ? "Successfully reached position and aligned orientation." : "Failed to align orientation.";
    }

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveToFrameServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



