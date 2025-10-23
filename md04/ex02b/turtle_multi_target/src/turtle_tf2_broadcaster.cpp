#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <turtlesim/msg/pose.hpp>
#include <cmath>

class TurtleTf2Broadcaster : public rclcpp::Node
{
public:
  TurtleTf2Broadcaster()
  : Node("turtle_tf2_broadcaster")
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    
    turtle1_sub_ = create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10,
      std::bind(&TurtleTf2Broadcaster::turtle1_pose_cb, this, std::placeholders::_1));
      
    turtle2_sub_ = create_subscription<turtlesim::msg::Pose>(
      "/turtle2/pose", 10,
      std::bind(&TurtleTf2Broadcaster::turtle2_pose_cb, this, std::placeholders::_1));
      
    turtle3_sub_ = create_subscription<turtlesim::msg::Pose>(
      "/turtle3/pose", 10,
      std::bind(&TurtleTf2Broadcaster::turtle3_pose_cb, this, std::placeholders::_1));
  }

private:
  void turtle1_pose_cb(const turtlesim::msg::Pose::SharedPtr msg)
  {
    publish_transform(*msg, "turtle1");
  }

  void turtle2_pose_cb(const turtlesim::msg::Pose::SharedPtr msg)
  {
    publish_transform(*msg, "turtle2");
  }

  void turtle3_pose_cb(const turtlesim::msg::Pose::SharedPtr msg)
  {
    publish_transform(*msg, "turtle3");
  }

  void publish_transform(const turtlesim::msg::Pose& pose, const std::string& turtle_name)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = turtle_name;
    
    t.transform.translation.x = pose.x;
    t.transform.translation.y = pose.y;
    t.transform.translation.z = 0.0;
    
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = sin(pose.theta / 2.0);
    t.transform.rotation.w = cos(pose.theta / 2.0);
    
    tf_broadcaster_->sendTransform(t);
  }

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle1_sub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle2_sub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle3_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleTf2Broadcaster>());
  rclcpp::shutdown();
  return 0;
}
