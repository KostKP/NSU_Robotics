#include "rclcpp/rclcpp.hpp"
#include "service_full_name/srv/summ_full_name.hpp"
#include <memory>

void summ_callback(
  const std::shared_ptr<service_full_name::srv::SummFullName::Request> request,
  std::shared_ptr<service_full_name::srv::SummFullName::Response> response)
{
  response->full_name = request->last_name + " " + request->first_name + " " + request->middle_name;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Full name: %s", response->full_name.c_str());
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("service_name");
  auto service = node->create_service<service_full_name::srv::SummFullName>(
    "SummFullName", &summ_callback);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service ready: SummFullName");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
