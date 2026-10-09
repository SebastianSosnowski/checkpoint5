#include "attach_shelf/srv/go_to_loading.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <vector>

class ApproachSrvServerNode : public rclcpp::Node {
public:
  ApproachSrvServerNode() : Node("approach_srv_server_node") {

    // Init command Publisher
    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Init elevator_up Publisher
    elevator_up_pub_ =
        this->create_publisher<std_msgs::msg::String>("/elevator_up", 10);

    // Subscribe to Laser Topic
    auto qos_laser =
        rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);
    subscriber_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", qos_laser, [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
          last_scan_ = msg;
        });

    // TF
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ =
        std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    // Create a service that will handle status queries
    std::string name_service = "/approach_shelf";
    service_ = this->create_service<attach_shelf::srv::GoToLoading>(
        name_service,
        [this](const std::shared_ptr<attach_shelf::srv::GoToLoading::Request>
                   request,
               std::shared_ptr<attach_shelf::srv::GoToLoading::Response>
                   response) { approach_callback(request, response); });

    RCLCPP_INFO(this->get_logger(), "%s Service Server Ready...",
                name_service.c_str());
  }

  // Machine state
private:
  enum class ApproachState { ROTATING, DRIVING, FORWARD_30CM, LIFTING, DONE };

  void final_approach() {
    rclcpp::Rate rate(20);
    ApproachState approach_state = ApproachState::ROTATING;
    while (true) {
      switch (approach_state) {
      case ApproachState::ROTATING: {
        auto cart_transform = get_cart_transform();
        if (!cart_transform) {
          break;
        }

        double angle = calculate_angle_to_cart(*cart_transform);
        constexpr double angle_tolerance = 0.01;

        geometry_msgs::msg::Twist cmd;

        if (std::abs(angle) < angle_tolerance) {
          cmd.angular.z = 0.0;
          approach_state = ApproachState::DRIVING;
          RCLCPP_INFO(this->get_logger(), "Rotated to the shelf, angle: %.2f",
                      angle);
        } else if (angle > 0.0) {
          cmd.angular.z = std::clamp(0.15, 0.5 * angle, 1.0);
        } else {
          cmd.angular.z = std::clamp(-1.0, 0.5 * angle, -0.15);
        }
        cmd_vel_pub_->publish(cmd);
        break;
      }

      case ApproachState::DRIVING: {
        auto cart_transform = get_cart_transform();

        if (!cart_transform) {
          break;
        }
        double x = cart_transform->transform.translation.x;

        constexpr double x_tolerance = 0.05;

        geometry_msgs::msg::Twist cmd;

        if (x < x_tolerance) {
          cmd.linear.x = 0.0;
          approach_state = ApproachState::FORWARD_30CM;
          RCLCPP_INFO(this->get_logger(), "Approached cart_frame!!");
        } else {
          cmd.linear.x = 0.3;
          RCLCPP_DEBUG(this->get_logger(), "Driving: x=%.3f", x);
        }
        cmd_vel_pub_->publish(cmd);
        break;
      }

      case ApproachState::FORWARD_30CM: {
        auto cart_transform = get_cart_transform();

        if (!cart_transform) {
          break;
        }
        double x = cart_transform->transform.translation.x;

        geometry_msgs::msg::Twist cmd;

        if (std::abs(x) >= 0.30) {
          cmd.linear.x = 0.0;
          approach_state = ApproachState::LIFTING;
          RCLCPP_INFO(this->get_logger(), "Moved forward 30cm!!");
        } else {
          cmd.linear.x = 0.3;
          RCLCPP_DEBUG(this->get_logger(), "Driving: x=%.3f", x);
        }
        cmd_vel_pub_->publish(cmd);
        break;
      }

      case ApproachState::LIFTING: {
        RCLCPP_INFO(this->get_logger(), "LIFTING!!");
        std_msgs::msg::String msg;
        elevator_up_pub_->publish(msg);
        approach_state = ApproachState::DONE;
        break;
      }
      case ApproachState::DONE: {
        RCLCPP_INFO(this->get_logger(), "Final approach done 😊!!");
        return;
      }
      }
      rate.sleep();
    }
  }

  // TF
private:
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

private:
  rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr service_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      subscriber_laser_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_up_pub_;

  std::vector<int> legs_idx_{};

  void approach_callback(
      const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
      std::shared_ptr<attach_shelf::srv::GoToLoading::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Approach Service Server Called!!");

    bool detected = detect_shelf_legs(*last_scan_);

    if (detected) {
      auto center = calculate_shelf_center_point(*last_scan_, legs_idx_);
      auto center_odom = transform_point_to_odom(center);
      if (!center_odom) {
        RCLCPP_ERROR(this->get_logger(),
                     "Could not transform shelf center to odom");
        response->complete = false;
        return;
      }
      publish_cart_frame(*center_odom);
      // If attach_to_shelf True, move towards shelf using cart_frame
      if (!request->attach_to_shelf) {
        response->complete = false;
        return;
      }
      final_approach();
      response->complete = true;
    } else {
      response->complete = false;
    }
  }

  double calculate_angle_to_cart(
      const geometry_msgs::msg::TransformStamped &transform) {
    double x = transform.transform.translation.x;
    double y = transform.transform.translation.y;

    return std::atan2(y, x);
  }

  std::optional<geometry_msgs::msg::TransformStamped> get_cart_transform() {
    try {
      auto transform = tf_buffer_->lookupTransform(
          "robot_base_link", "cart_frame", tf2::TimePointZero);

      return transform;

    } catch (const tf2::TransformException &ex) {

      RCLCPP_WARN(this->get_logger(),
                  "Could not transform cart_frame to robot_base_link: %s",
                  ex.what());

      return std::nullopt;
    }
  }

  void publish_cart_frame(const geometry_msgs::msg::PointStamped &center_odom) {
    geometry_msgs::msg::TransformStamped transform;

    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = "odom";
    transform.child_frame_id = "cart_frame";

    transform.transform.translation.x = center_odom.point.x;
    transform.transform.translation.y = center_odom.point.y;
    transform.transform.translation.z = 0.0;

    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(transform);
    RCLCPP_INFO(this->get_logger(), "Cart Frame Created!");
  }

  std::optional<geometry_msgs::msg::PointStamped>
  transform_point_to_odom(const geometry_msgs::msg::PointStamped &point) {
    std::string fixed_frame = "odom";
    try {
      auto transform = tf_buffer_->lookupTransform(
          fixed_frame, point.header.frame_id, tf2::TimePointZero);
      RCLCPP_DEBUG(this->get_logger(), "Transform %s -> %s",
                   point.header.frame_id.c_str(), fixed_frame.c_str());

      RCLCPP_DEBUG(this->get_logger(), "Position: x=%.3f, y=%.3f, z=%.3f",
                   transform.transform.translation.x,
                   transform.transform.translation.y,
                   transform.transform.translation.z);
      RCLCPP_DEBUG(
          this->get_logger(), "Rotation: x=%.3f, y=%.3f, z=%.3f, w=%.3f",
          transform.transform.rotation.x, transform.transform.rotation.y,
          transform.transform.rotation.z, transform.transform.rotation.w);

      geometry_msgs::msg::PointStamped point_odom;
      tf2::doTransform(point, point_odom, transform);
      RCLCPP_DEBUG(this->get_logger(),
                   "Shelf center in odom: x=%.3f, y=%.3f, z=%.3f",
                   point_odom.point.x, point_odom.point.y, point_odom.point.z);
      RCLCPP_DEBUG(this->get_logger(), "Shelf center frame: %s",
                   point_odom.header.frame_id.c_str());
      return point_odom;

    } catch (const tf2::TransformException &ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                  point.header.frame_id.c_str(), fixed_frame.c_str(),
                  ex.what());
      return std::nullopt;
    }
  }

  geometry_msgs::msg::PointStamped
  calculate_shelf_center_point(const sensor_msgs::msg::LaserScan &msg,
                               const std::vector<int> &legs_idx) {
    auto leg1 = calculate_leg_position(legs_idx[0], msg);
    auto leg2 = calculate_leg_position(legs_idx[1], msg);

    geometry_msgs::msg::PointStamped center_point{};

    center_point.header.frame_id = msg.header.frame_id;
    center_point.header.stamp = msg.header.stamp;
    center_point.point.x = (leg1.x + leg2.x) / 2.0;
    center_point.point.y = (leg1.y + leg2.y) / 2.0;
    center_point.point.z = 0.0;

    return center_point;
  }

  geometry_msgs::msg::Point
  calculate_leg_position(int scan_idx, const sensor_msgs::msg::LaserScan &msg) {
    double angle = msg.angle_min + scan_idx * msg.angle_increment;

    double range = msg.ranges[scan_idx];

    geometry_msgs::msg::Point pos{};
    pos.x = range * std::cos(angle);
    pos.y = range * std::sin(angle);
    return pos;
  }

  bool detect_shelf_legs(const sensor_msgs::msg::LaserScan &msg) {
    std::vector<std::vector<int>> groups{};
    std::vector<int> current_group{};
    legs_idx_.clear();

    for (size_t i = 0; i < msg.intensities.size(); ++i) {
      if (msg.intensities[i] > 7800) {
        current_group.push_back(i);
      } else if (!current_group.empty()) {
        groups.push_back(current_group);
        current_group.clear();
      }
    }
    // in case we detect leg in the last index
    if (!current_group.empty()) {
      groups.push_back(current_group);
      current_group.clear();
    }

    if (groups.size() != 2) {
      return false;
    }
    for (const auto &group : groups) {
      int middle_index = group[group.size() / 2];
      legs_idx_.push_back(middle_index);
    }
    return true;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ApproachSrvServerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}