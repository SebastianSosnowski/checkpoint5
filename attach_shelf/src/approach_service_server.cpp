#include "attach_shelf/srv/go_to_loading.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>

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
  rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr service_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      subscriber_laser_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
  std::vector<int> legs_idx_{};

  void approach_callback(
      const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
      std::shared_ptr<attach_shelf::srv::GoToLoading::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Service Server Called!!");
    // Detect legs of the shelf
    bool detected = detect_shelf_legs(*last_scan_);

    if (detected) {
      // public cart_frame transform
      auto center = calculate_shelf_center(*last_scan_, legs_idx_);

      public_cart_frame(center);
      // If attach_to_shelf True, move towards shelf using cart_frame
      // After reaching tf coordinates, move 30 cm more
      // Lift shelf
      response->complete = true;
    } else {
      response->complete = false;
    }
  }

  geometry_msgs::msg::Point
  calculate_shelf_center(const sensor_msgs::msg::LaserScan &msg,
                         const std::vector<int> &legs_idx) {
    auto leg1 = calculate_leg_position(legs_idx[0], msg);
    auto leg2 = calculate_leg_position(legs_idx[1], msg);
    geometry_msgs::msg::Point center_point{};
    center_point.x = (leg1.x + leg2.x) / 2.0;
    center_point.y = (leg1.y + leg2.y) / 2.0;
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

  void public_cart_frame(geometry_msgs::msg::Point &center) {
    for (int index : legs_idx_) {
      RCLCPP_INFO(this->get_logger(), "leg index: %d", index);
    }
    RCLCPP_INFO(this->get_logger(), "Shelf center: x=%.3f, y=%.3f", center.x,
                center.y);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ApproachSrvServerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}