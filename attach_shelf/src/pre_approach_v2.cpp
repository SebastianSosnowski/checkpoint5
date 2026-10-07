#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <cmath>
#include <string>

using namespace std::chrono_literals;

enum class PreApproachState { MOVE, ROTATE, STOP };

class PreApproachNode : public rclcpp_lifecycle::LifecycleNode {
public:
  PreApproachNode() : rclcpp_lifecycle::LifecycleNode("pre_approach_node") {
    // Parameter descriptors
    rcl_interfaces::msg::ParameterDescriptor obstacle_desc;
    obstacle_desc.description =
        "Distance (in meters) to the obstacle at which the robot will stop.";

    rcl_interfaces::msg::ParameterDescriptor degrees_desc;
    degrees_desc.description =
        "Number of degrees for the rotation of the robot after stopping.";

    rcl_interfaces::msg::ParameterDescriptor final_approach_desc;
    final_approach_desc.description =
        "Decide whether the robot will do the final approach or not.";

    // Declare parameters (typed, with defaults)
    this->declare_parameter<double>("obstacle", 0.1, obstacle_desc);
    this->declare_parameter<int>("degrees", 90, degrees_desc);
    this->declare_parameter<bool>("final_approach", false, final_approach_desc);

    RCLCPP_INFO(this->get_logger(), "Node created. Currently unconfigured");
  }

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(this->get_logger(), "Configuring node...");

    // Read parameters once at startup
    obstacle_ = this->get_parameter("obstacle").as_double();
    degrees_ = this->get_parameter("degrees").as_int();
    final_approach_ = this->get_parameter("final_approach").as_bool();

    if (obstacle_ <= 0.0) {
      RCLCPP_ERROR(this->get_logger(), "obstacle must be greater than 0.0");
      return CallbackReturn::FAILURE;
    }

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
    RCLCPP_INFO(this->get_logger(), "Node configured successfully.");

    // Create the Service Client object
    std::string name_service = "/approach_shelf";
    client_ = this->create_client<attach_shelf::srv::GoToLoading>(name_service);
    // Wait for the service to be available (checks every second)
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for the service. Exiting.");
        return CallbackReturn::FAILURE;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Service %s not available, waiting again...",
                  name_service.c_str());
    }

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(this->get_logger(), "Activating node...");
    command_publisher_->on_activate();
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(this->get_logger(), "Deactivating node...");
    command_publisher_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(this->get_logger(), "Cleaning up node...");
    timer_.reset();
    command_publisher_.reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(this->get_logger(), "Shutting down node...");
    timer_.reset();
    command_publisher_.reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_error(const rclcpp_lifecycle::State &) {
    RCLCPP_ERROR(this->get_logger(),
                 "An error occurred. Cleaning up resources...");

    timer_.reset();
    command_publisher_.reset();

    return CallbackReturn::SUCCESS;
  }

private:
  rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedPtr client_;
  bool approach_started_ = false;
  bool final_approach_;

private:
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr
      command_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  PreApproachState pre_approach_state_ = PreApproachState::MOVE;
  bool front_wall_ = false;
  double obstacle_;
  int degrees_;

  void pre_approach_callback() {
    if (!command_publisher_ || !command_publisher_->is_activated()) {
      return;
    }

    auto action = geometry_msgs::msg::Twist();
    switch (pre_approach_state_) {
    case PreApproachState::MOVE: {
      RCLCPP_INFO(this->get_logger(), "State Move");
      if (front_wall_) {
        action.linear.x = 0.0;
        pre_approach_state_ = PreApproachState::ROTATE;
        //  read current pos as init pos
        double angle_rad = degrees_ * M_PI / 180.0;
        target_yaw_ = tf2NormalizeAngle(current_yaw_ + angle_rad);
      } else {
        action.linear.x = 1.0;
      }
      break;
    }
    case PreApproachState::ROTATE: {
      RCLCPP_INFO(this->get_logger(), "State Rotate");
      double error = tf2NormalizeAngle(target_yaw_ - current_yaw_);
      if (std::abs(error) < 0.05) {
        action.angular.z = 0.0;
        pre_approach_state_ = PreApproachState::STOP;
      } else if (error > 0.0) {
        action.angular.z = 0.3;
      } else {
        action.angular.z = -0.3;
      }
      break;
    }
    case PreApproachState::STOP: {
      RCLCPP_INFO(this->get_logger(), "State Stop");

      action.linear.x = 0.0;
      action.angular.z = 0.0;
      command_publisher_->publish(action);

      if (!approach_started_) {
        approach_started_ = true;

        auto request =
            std::make_shared<attach_shelf::srv::GoToLoading::Request>();
        request->attach_to_shelf = final_approach_;

        client_->async_send_request(
            request,
            [this](rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedFuture
                       future) {
              auto response = future.get();

              if (response->complete) {
                RCLCPP_INFO(this->get_logger(), "Approach completed");

                trigger_transition(lifecycle_msgs::msg::Transition::
                                       TRANSITION_ACTIVE_SHUTDOWN);

                rclcpp::shutdown();
              } else {
                RCLCPP_ERROR(this->get_logger(), "Approach failed");
              }
            });
      }

      break;
    }
    }
    command_publisher_->publish(action);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      subscriber_laser_;

  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    int front_index =
        static_cast<int>((0.0 - msg->angle_min) / msg->angle_increment);

    if (front_index < 0 ||
        front_index >= static_cast<int>(msg->ranges.size())) {
      RCLCPP_WARN(this->get_logger(), "Front laser index out of range");
      return;
    }

    double distance = msg->ranges[front_index];

    front_wall_ = std::isfinite(distance) && distance < obstacle_;

    RCLCPP_DEBUG(this->get_logger(),
                 "distance[%d] = %.3f, obstacle = %.3f, front_wall = %s",
                 front_index, distance, obstacle_,
                 front_wall_ ? "TRUE" : "FALSE");
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  double current_yaw_ = std::numeric_limits<double>::infinity();
  double target_yaw_ = std::numeric_limits<double>::infinity();

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
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
  node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}