#include "rclcpp/rclcpp.hpp"
#include "service_full_name/srv/summ_full_name.hpp"
#include <memory>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 4) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Usage: ros2 run service_full_name client_name <Last> <First> <Middle>");
    return 1;
  }

  auto node = rclcpp::Node::make_shared("client_name");
  auto client = node->create_client<service_full_name::srv::SummFullName>("SummFullName");

  auto request = std::make_shared<service_full_name::srv::SummFullName::Request>();
  request->last_name = argv[1];
  request->first_name = argv[2];
  request->middle_name = argv[3];

  while (!client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for service...");
  }

  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Full name: %s", result.get()->full_name.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service call failed");
  }

  rclcpp::shutdown();
  return 0;
}
