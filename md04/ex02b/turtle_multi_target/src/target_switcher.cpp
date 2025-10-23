#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class TargetSwitcher : public rclcpp::Node {
public:
  TargetSwitcher(): Node("target_switcher") {
    this->declare_parameter<double>("radius1", 1.0);
    this->declare_parameter<double>("radius2", 1.5);
    this->declare_parameter<int>("direction_of_rotation1", 1);
    this->declare_parameter<int>("direction_of_rotation2", 1);
    this->declare_parameter<double>("static_x", 8.0);
    this->declare_parameter<double>("static_y", 2.0);
    this->declare_parameter<double>("switch_threshold", 1.0);

    radius1_ = this->get_parameter("radius1").as_double();
    radius2_ = this->get_parameter("radius2").as_double();
    dir1_ = (this->get_parameter("direction_of_rotation1").as_int() >= 0) ? 1 : -1;
    dir2_ = (this->get_parameter("direction_of_rotation2").as_int() >= 0) ? 1 : -1;
    static_x_ = this->get_parameter("static_x").as_double();
    static_y_ = this->get_parameter("static_y").as_double();
    switch_threshold_ = this->get_parameter("switch_threshold").as_double();

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    static_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    timer_ = this->create_wall_timer(50ms, std::bind(&TargetSwitcher::on_timer, this));
  }

private:
  void on_timer() {
    auto now = this->get_clock()->now();

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now;
    // carrot1 relative to turtle1
    t.header.frame_id = "turtle1";
    t.child_frame_id = "carrot1";
    t.transform.translation.x = radius1_ * cos(angle1_);
    t.transform.translation.y = radius1_ * sin(angle1_);
    t.transform.translation.z = 0.0;
    tf2::Quaternion q; q.setRPY(0,0,0);
    t.transform.rotation.x = q.x(); t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z(); t.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(t);

    // carrot2 relative to turtle3
    geometry_msgs::msg::TransformStamped t2;
    t2.header.stamp = now;
    t2.header.frame_id = "turtle3";
    t2.child_frame_id = "carrot2";
    t2.transform.translation.x = radius2_ * cos(angle2_);
    t2.transform.translation.y = radius2_ * sin(angle2_);
    t2.transform.translation.z = 0.0;
    tf_broadcaster_->sendTransform(t2);

    // static_target in world/frame "world"
    geometry_msgs::msg::TransformStamped st;
    st.header.stamp = now;
    st.header.frame_id = "world";
    st.child_frame_id = "static_target";
    st.transform.translation.x = static_x_;
    st.transform.translation.y = static_y_;
    st.transform.translation.z = 0.0;
    tf2::Quaternion qs; qs.setRPY(0,0,0);
    st.transform.rotation.x = qs.x(); st.transform.rotation.y = qs.y();
    st.transform.rotation.z = qs.z(); st.transform.rotation.w = qs.w();
    static_broadcaster_->sendTransform(st);

    angle1_ += 0.08 * dir1_;
    angle2_ += 0.06 * dir2_;
  }

  // параметры
  double radius1_{1.0}, radius2_{1.0}, static_x_{8.0}, static_y_{2.0};
  int dir1_{1}, dir2_{1};
  double angle1_{0.0}, angle2_{0.0};
  double switch_threshold_{1.0};

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> static_broadcaster_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TargetSwitcher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
