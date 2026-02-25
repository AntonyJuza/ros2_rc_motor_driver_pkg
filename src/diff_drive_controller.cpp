#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>

class DiffDriveController : public rclcpp::Node
{
public:
  DiffDriveController()
  : Node("diff_drive_controller"),
    x_(0.0), y_(0.0), theta_(0.0)
  {
    // Declare parameters
    this->declare_parameter("wheel_base", 0.4);
    this->declare_parameter("wheel_radius", 0.05);
    this->declare_parameter("publish_tf", true);
    this->declare_parameter("base_frame", "base_link");
    this->declare_parameter("odom_frame", "odom");

    // Get parameters
    wheel_base_ = this->get_parameter("wheel_base").as_double();
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();
    base_frame_ = this->get_parameter("base_frame").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();

    // Create subscribers
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      std::bind(&DiffDriveController::jointStateCallback, this, std::placeholders::_1));

    // Create publishers
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    // TF broadcaster
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    }

    last_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Differential drive controller started");
  }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->velocity.size() < 4) {
      return;
    }

    auto current_time = this->now();
    double dt = (current_time - last_time_).seconds();

    if (dt <= 0) {
      return;
    }

    // Average left and right wheel velocities
    // Assuming wheel order: front_left, front_right, rear_left, rear_right
    double left_vel = (msg->velocity[0] + msg->velocity[2]) / 2.0;  // Average of left wheels
    double right_vel = (msg->velocity[1] + msg->velocity[3]) / 2.0;  // Average of right wheels

    // Calculate robot velocities using differential drive kinematics
    double linear_vel = (left_vel + right_vel) / 2.0;
    double angular_vel = (right_vel - left_vel) / wheel_base_;

    // Update odometry using Runge-Kutta 2nd order integration
    double delta_theta = angular_vel * dt;
    double delta_x = linear_vel * cos(theta_ + delta_theta / 2.0) * dt;
    double delta_y = linear_vel * sin(theta_ + delta_theta / 2.0) * dt;

    x_ += delta_x;
    y_ += delta_y;
    theta_ += delta_theta;

    // Normalize theta to [-pi, pi]
    while (theta_ > M_PI) theta_ -= 2.0 * M_PI;
    while (theta_ < -M_PI) theta_ += 2.0 * M_PI;

    // Create and publish odometry message
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    // Position
    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;

    // Orientation (quaternion from yaw)
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    // Velocity
    odom_msg.twist.twist.linear.x = linear_vel;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = angular_vel;

    // Covariance (tune these values based on your encoder accuracy)
    odom_msg.pose.covariance[0] = 0.001;  // x
    odom_msg.pose.covariance[7] = 0.001;  // y
    odom_msg.pose.covariance[35] = 0.01;  // theta
    odom_msg.twist.covariance[0] = 0.001;  // vx
    odom_msg.twist.covariance[35] = 0.01;  // vtheta

    odom_pub_->publish(odom_msg);

    // Publish TF transform
    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = current_time;
      tf_msg.header.frame_id = odom_frame_;
      tf_msg.child_frame_id = base_frame_;

      tf_msg.transform.translation.x = x_;
      tf_msg.transform.translation.y = y_;
      tf_msg.transform.translation.z = 0.0;

      tf_msg.transform.rotation.x = q.x();
      tf_msg.transform.rotation.y = q.y();
      tf_msg.transform.rotation.z = q.z();
      tf_msg.transform.rotation.w = q.w();

      tf_broadcaster_->sendTransform(tf_msg);
    }

    last_time_ = current_time;
  }

  // Odometry state
  double x_, y_, theta_;
  
  // Parameters
  double wheel_base_;
  double wheel_radius_;
  bool publish_tf_;
  std::string base_frame_;
  std::string odom_frame_;

  // ROS2 interfaces
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Time last_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DiffDriveController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}