#include "humanoid_motor_control/cytron_controller.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <algorithm>  // For std::clamp

using humanoid_motor_control::CytronController;

class CytronNode : public rclcpp::Node
{
public:
    CytronNode()
    : Node("cytron_controller_node"), emergency_stop_(false)
    {
        // Parameters for motor pins (GPIO 12 and 13 support hardware PWM!)
        int rc1 = this->declare_parameter("rc1_pin", 12);
        int rc2 = this->declare_parameter("rc2_pin", 13);
        
        // Initialize controller
        if (!controller_.init(rc1, rc2)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize CytronController");
            RCLCPP_ERROR(this->get_logger(), "Make sure pigpiod is running: sudo pigpiod");
            rclcpp::shutdown();
        } else {
            RCLCPP_INFO(this->get_logger(), "CytronController initialized on RC1=GPIO%d RC2=GPIO%d", rc1, rc2);
            RCLCPP_INFO(this->get_logger(), "Using hardware PWM channels - excellent!");
        }
        
        // Publisher for wheel velocities (for debugging/monitoring)
        wheel_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/wheel_velocities", 10);
        
        // Subscriber for /cmd_vel
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&CytronNode::cmdVelCallback, this, std::placeholders::_1));
        
        // Subscriber for emergency stop
        estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/emergency_stop", 10,
            std::bind(&CytronNode::estopCallback, this, std::placeholders::_1));
        
        // Maximum linear speed (m/s) and angular speed (rad/s) for mapping
        max_linear_speed_ = this->declare_parameter("max_linear_speed", 0.5);
        max_angular_speed_ = this->declare_parameter("max_angular_speed", 1.0);
        wheel_base_ = this->declare_parameter("wheel_base", 0.25); // meters
        
        RCLCPP_INFO(this->get_logger(), "Configuration:");
        RCLCPP_INFO(this->get_logger(), "  Max linear speed: %.2f m/s", max_linear_speed_);
        RCLCPP_INFO(this->get_logger(), "  Max angular speed: %.2f rad/s", max_angular_speed_);
        RCLCPP_INFO(this->get_logger(), "  Wheel base: %.3f m", wheel_base_);
    }
    
    ~CytronNode()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down - stopping motors");
        controller_.emergencyStop();
        controller_.close();
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Check emergency stop
        if (emergency_stop_) {
            return;
        }
        
        // Extract linear and angular velocities
        double linear = msg->linear.x;
        double angular = msg->angular.z;
        
        // Differential drive kinematics: convert to left/right wheel velocities
        // left_vel = linear - (angular * wheel_base / 2)
        // right_vel = linear + (angular * wheel_base / 2)
        double left_vel = linear - (angular * wheel_base_ / 2.0);
        double right_vel = linear + (angular * wheel_base_ / 2.0);
        
        // Map to motor speed percentage (-100 to 100)
        int left_speed = static_cast<int>(100.0 * left_vel / max_linear_speed_);
        int right_speed = static_cast<int>(100.0 * right_vel / max_linear_speed_);
        
        // Clamp speeds to valid range [-100, 100]
        left_speed = std::clamp(left_speed, -100, 100);
        right_speed = std::clamp(right_speed, -100, 100);
        
        // Send speeds to CytronController
        controller_.setLeftRight(left_speed, right_speed);
        
        // Log for debugging (only when non-zero to avoid spam)
        if (left_speed != 0 || right_speed != 0) {
            RCLCPP_DEBUG(this->get_logger(), 
                        "cmd_vel: lin=%.2f ang=%.2f → L=%d%% R=%d%%",
                        linear, angular, left_speed, right_speed);
        }
        
        // Publish wheel velocities for monitoring
        auto wheel_msg = geometry_msgs::msg::Twist();
        wheel_msg.linear.x = static_cast<double>(left_speed);
        wheel_msg.linear.y = static_cast<double>(right_speed);
        wheel_vel_pub_->publish(wheel_msg);
    }
    
    void estopCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        emergency_stop_ = msg->data;
        
        if (emergency_stop_) {
            RCLCPP_WARN(this->get_logger(), "🚨 EMERGENCY STOP ACTIVATED!");
            controller_.emergencyStop();
        } else {
            RCLCPP_INFO(this->get_logger(), "✅ Emergency stop deactivated");
        }
    }
    
    CytronController controller_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr wheel_vel_pub_;
    
    double max_linear_speed_;
    double max_angular_speed_;
    double wheel_base_;
    bool emergency_stop_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<CytronNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}