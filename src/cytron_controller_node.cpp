#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <algorithm>
#include <cmath>

#include "humanoid_motor_control/cytron_controller.hpp"

using humanoid_motor_control::CytronController;

class CytronNode : public rclcpp::Node
{
public:
    CytronNode() : Node("cytron_controller"), emergency_stop_(false)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing Cytron Motor Controller Node...");
        
        int rc1_pin = this->declare_parameter("rc1_pin", 12);
        int rc2_pin = this->declare_parameter("rc2_pin", 13);
        
        if (!controller_.init(rc1_pin, rc2_pin)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize Cytron controller!");
            return;
        }
        
        left_trim_ = this->declare_parameter("left_trim", 1.0);
        right_trim_ = this->declare_parameter("right_trim", 1.0);
        trim_ = this->declare_parameter("trim", 0.0);
        updateTrims();

        max_linear_speed_ = this->declare_parameter("max_linear_speed", 0.15);
        max_angular_speed_ = this->declare_parameter("max_angular_speed", 0.80);
        min_speed_ = this->declare_parameter("min_speed", 25);
        wheel_base_ = this->declare_parameter("wheel_base", 0.28);

        param_callback_handle_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> & params) {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                bool trim_changed = false;
                for (const auto & param : params) {
                    if (param.get_name() == "left_trim") {
                        left_trim_ = param.as_double();
                        trim_changed = true;
                    } else if (param.get_name() == "right_trim") {
                        right_trim_ = param.as_double();
                        trim_changed = true;
                    } else if (param.get_name() == "trim") {
                        trim_ = param.as_double();
                        trim_changed = true;
                    } else if (param.get_name() == "max_linear_speed") {
                        max_linear_speed_ = param.as_double();
                        RCLCPP_INFO(this->get_logger(), "Updated max_linear_speed: %.2f m/s", max_linear_speed_);
                    } else if (param.get_name() == "min_speed") {
                        min_speed_ = param.as_int();
                        RCLCPP_INFO(this->get_logger(), "Updated min_speed (deadband): %d%%", min_speed_);
                    } else if (param.get_name() == "wheel_base") {
                        wheel_base_ = param.as_double();
                        RCLCPP_INFO(this->get_logger(), "Updated wheel_base: %.3f m", wheel_base_);
                    }
                }
                if (trim_changed) {
                    updateTrims();
                }
                return result;
            });

        wheel_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/wheel_velocities", 10);
        
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&CytronNode::cmdVelCallback, this, std::placeholders::_1));
            
        estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/emergency_stop", 10,
            std::bind(&CytronNode::estopCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Configuration:");
        RCLCPP_INFO(this->get_logger(), "  Max linear speed: %.2f m/s", max_linear_speed_);
        RCLCPP_INFO(this->get_logger(), "  Max angular speed: %.2f rad/s", max_angular_speed_);
        RCLCPP_INFO(this->get_logger(), "  Min speed (deadband): %d%%", min_speed_);
        RCLCPP_INFO(this->get_logger(), "  Wheel base: %.3f m", wheel_base_);
        RCLCPP_INFO(this->get_logger(), "  Motor trim: left=%.3f, right=%.3f, diff_trim=%.3f",
                    left_trim_, right_trim_, trim_);
    }
    
    ~CytronNode()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down - stopping motors");
        controller_.emergencyStop();
        controller_.close();
    }

private:
    void updateTrims()
    {
        double eff_left = left_trim_;
        double eff_right = right_trim_;
        if (trim_ > 0.0) {
            eff_left *= (1.0 - std::min(trim_, 1.0));
        } else if (trim_ < 0.0) {
            eff_right *= (1.0 - std::min(-trim_, 1.0));
        }
        controller_.setTrim(eff_left, eff_right);
        RCLCPP_INFO(this->get_logger(),
                    "Motor trims applied -> Left: %.3f (eff: %.3f), Right: %.3f (eff: %.3f), Diff Trim: %.3f",
                    left_trim_, eff_left, right_trim_, eff_right, trim_);
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (emergency_stop_) {
            return;
        }
        
        double linear = msg->linear.x;
        double angular = msg->angular.z;
        
        double left_vel = linear - (angular * wheel_base_ / 2.0);
        double right_vel = linear + (angular * wheel_base_ / 2.0);
        
        int left_speed = static_cast<int>(100.0 * left_vel / max_linear_speed_);
        int right_speed = static_cast<int>(100.0 * right_vel / max_linear_speed_);
        
        // Deadband compensation: ensure motor receives enough starting torque
        if (left_speed > 0 && left_speed < min_speed_) left_speed = min_speed_;
        if (left_speed < 0 && left_speed > -min_speed_) left_speed = -min_speed_;
        if (right_speed > 0 && right_speed < min_speed_) right_speed = min_speed_;
        if (right_speed < 0 && right_speed > -min_speed_) right_speed = -min_speed_;
        
        left_speed = std::clamp(left_speed, -100, 100);
        right_speed = std::clamp(right_speed, -100, 100);
        
        controller_.setLeftRight(left_speed, right_speed);
        
        int eff_left_speed = std::clamp(static_cast<int>(std::round(left_speed * controller_.getLeftTrim())), -100, 100);
        int eff_right_speed = std::clamp(static_cast<int>(std::round(right_speed * controller_.getRightTrim())), -100, 100);

        auto wheel_msg = geometry_msgs::msg::Twist();
        wheel_msg.linear.x = static_cast<double>(eff_left_speed);
        wheel_msg.linear.y = static_cast<double>(eff_right_speed);
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
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    
    double max_linear_speed_;
    double max_angular_speed_;
    int min_speed_;
    double wheel_base_;
    double left_trim_;
    double right_trim_;
    double trim_;
    bool emergency_stop_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CytronNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
