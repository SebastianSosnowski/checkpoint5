#include "attach_shelf/srv/go_to_loading.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <optional>
#include <vector>

class ApproachSrvServerNode : public rclcpp::Node {
public:
  ApproachSrvServerNode() : Node("approach_srv_server_node") {

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

private:
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

private:
  rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr service_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      subscriber_laser_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;

  std::vector<int> legs_idx_{};

  void approach_callback(
      const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
      std::shared_ptr<attach_shelf::srv::GoToLoading::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Service Server Called!!");

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
      // After reaching tf coordinates, move 30 cm more
      // Lift shelf
      response->complete = true;
    } else {
      response->complete = false;
    }
  }

  void publish_cart_frame(const geometry_msgs::msg::PointStamped &center_odom) {
    geometry_msgs::msg::TransformStamped transform;

    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = "odom";
    transform.child_frame_id = "cart_frame";

    transform.transform.translation.x = center_odom.point.x;
    transform.transform.translation.y = center_odom.point.y;
    transform.transform.translation.z = center_odom.point.z;

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
      RCLCPP_INFO(this->get_logger(), "Transform %s -> %s",
                  point.header.frame_id.c_str(), fixed_frame.c_str());

      RCLCPP_INFO(this->get_logger(), "Position: x=%.3f, y=%.3f, z=%.3f",
                  transform.transform.translation.x,
                  transform.transform.translation.y,
                  transform.transform.translation.z);
      RCLCPP_INFO(
          this->get_logger(), "Rotation: x=%.3f, y=%.3f, z=%.3f, w=%.3f",
          transform.transform.rotation.x, transform.transform.rotation.y,
          transform.transform.rotation.z, transform.transform.rotation.w);

      geometry_msgs::msg::PointStamped point_odom;
      tf2::doTransform(point, point_odom, transform);
      RCLCPP_INFO(this->get_logger(),
                  "Shelf center in odom: x=%.3f, y=%.3f, z=%.3f",
                  point_odom.point.x, point_odom.point.y, point_odom.point.z);
      RCLCPP_INFO(this->get_logger(), "Shelf center frame: %s",
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