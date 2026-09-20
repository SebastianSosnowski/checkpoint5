#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <cmath>
#include <string>

enum class PreApproachState { MOVE, ROTATE, STOP };

class PreApproachNode : public rclcpp::Node {
public:
  PreApproachNode() : Node("pre_approach_node") {
    // Subscribe to Odometry Topic
    auto qos_odom =
        rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", qos_odom, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
          this->odom_callback(msg);
        });
    // Subscribe to Laser Topic
    auto qos_laser =
        rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);
    subscriber_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", qos_laser, [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
          this->laserscan_callback(msg);
        });
    // Init command Publisher
    command_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    auto timer_period = std::chrono::milliseconds(100);
    timer_ = this->create_wall_timer(timer_period,
                                     [this] { pre_approach_callback(); });

    RCLCPP_INFO(this->get_logger(), "Node started");
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  PreApproachState pre_approach_state_ = PreApproachState::MOVE;

  void pre_approach_callback() {
    // State Move
    // if detect wall x meters in front:
    //  stop cmd
    //  read current pos as init pos
    //  change state to Rotate
    // else move forward cmd

    // State Rotate
    // if current yaw != desired yaw
    //  publish rotation cmd
    // else
    //  stop rotation cmd
    //  change state to finish

    // State Finish
    //  Do nothing -> end task

    ;
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      subscriber_laser_;

  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    ;
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  double current_yaw_ = std::numeric_limits<double>::infinity();
  geometry_msgs::msg::Point current_position_{};
  geometry_msgs::msg::Point target_position_{};
  bool initial_position_received_ = false;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Update the current position and yaw from odometry
    current_position_.x = msg->pose.pose.position.x;
    current_position_.y = msg->pose.pose.position.y;

    if (!initial_position_received_) {

      initial_position_received_ = true;
    }

    // Extract yaw from quaternion
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, current_yaw_);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PreApproachNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}