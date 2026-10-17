#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"

using std::placeholders::_1;

class PoseListener : public rclcpp::Node
{
public:
  PoseListener()
  : Node("pose_listener")
  {
    pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10, std::bind(&PoseListener::pose_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Listening to /turtle1/pose ...");
  }

private:
  void pose_callback(const turtlesim::msg::Pose & pose)
  {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "x=%.2f  y=%.2f  theta=%.2f rad  | linear_vel=%.2f  angular_vel=%.2f",
      pose.x, pose.y, pose.theta, pose.linear_velocity, pose.angular_velocity);
  }

  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseListener>());
  rclcpp::shutdown();
  return 0;
}