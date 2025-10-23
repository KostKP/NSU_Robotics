#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "turtle_multi_target/msg/current_target.hpp"

using namespace std::chrono_literals;

int kbhit() {
  struct termios oldt, newt;
  int ch;
  int oldf;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
  if(ch != EOF){
    ungetc(ch, stdin);
    return 1;
  }
  return 0;
}

char getch_nonblock() {
  int c = getchar();
  if (c == EOF) return 0;
  return static_cast<char>(c);
}

class TurtleController : public rclcpp::Node {
public:
  TurtleController(): Node("turtle_controller"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
    pub_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);
    pub_current_ = this->create_publisher<turtle_multi_target::msg::CurrentTarget>("/current_target", 10);

    this->declare_parameter<double>("switch_threshold", 1.0);
    switch_threshold_ = this->get_parameter("switch_threshold").as_double();

    targets_ = {"carrot1","carrot2","static_target"};
    current_idx_ = 0; // start with carrot1

    timer_ = this->create_wall_timer(100ms, std::bind(&TurtleController::on_timer, this));
  }

private:
  void on_timer() {
    // check for 'n' key
    if (kbhit()) {
      char c = getch_nonblock();
      if (c == 'n' || c == 'N') {
        next_target_manual();
      }
    }

    std::string cur = targets_[current_idx_];

    try {
      auto now = this->get_clock()->now();
      // получаем трансформацию из turtle2 -> target в локальной системе turtle2
      geometry_msgs::msg::TransformStamped t = tf_buffer_.lookupTransform("turtle2", cur, tf2::TimePointZero);

      double dx = t.transform.translation.x;
      double dy = t.transform.translation.y;
      double dist = std::hypot(dx, dy);
      double angle = std::atan2(dy, dx);

      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = std::min(2.0, 1.5 * dist);
      cmd.angular.z = 4.0 * angle;
      pub_vel_->publish(cmd);

      // publish current target msg (get coordinates in world)
      turtle_multi_target::msg::CurrentTarget ct;
      ct.target_name = cur;

      try {
        auto world_t = tf_buffer_.lookupTransform("world", cur, tf2::TimePointZero);
        ct.target_x = world_t.transform.translation.x;
        ct.target_y = world_t.transform.translation.y;
      } catch (const std::exception & ex) {
        ct.target_x = 0.0; ct.target_y = 0.0;
      }

      ct.distance_to_target = dist;
      pub_current_->publish(ct);

      // automatic switch
      if (dist < switch_threshold_) {
        next_target_auto();
      }

    } catch (const tf2::TransformException & ex) {
      // трансформация недоступна — ничего не делаем
      RCLCPP_DEBUG(this->get_logger(), "Transform not available: %s", ex.what());
      return;
    }
  }

  void next_target_auto() {
    current_idx_ = (current_idx_ + 1) % targets_.size();
    RCLCPP_INFO(this->get_logger(), "AUTO SWITCH -> new target: %s", targets_[current_idx_].c_str());
  }

  void next_target_manual() {
    current_idx_ = (current_idx_ + 1) % targets_.size();
    RCLCPP_INFO(this->get_logger(), "MANUAL SWITCH (n) -> new target: %s", targets_[current_idx_].c_str());
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
  rclcpp::Publisher<turtle_multi_target::msg::CurrentTarget>::SharedPtr pub_current_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::vector<std::string> targets_;
  size_t current_idx_;
  double switch_threshold_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TurtleController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
