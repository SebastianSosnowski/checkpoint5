#include "attach_shelf/srv/go_to_loading.hpp"

#include <rclcpp/rclcpp.hpp>

class ApproachSrvServerNode : public rclcpp::Node {
public:
  ApproachSrvServerNode() : Node("approach_srv_server_node") {

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

  void approach_callback(
      const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
      std::shared_ptr<attach_shelf::srv::GoToLoading::Response> response) {
    RCLCPP_INFO(this->get_logger(), "%s Service Server Called!!",
                name_service.c_str());
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ApproachSrvServerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}