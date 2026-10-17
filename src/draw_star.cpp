#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"

using std::placeholders::_1;

class StarDriver : public rclcpp::Node
{
public:
  StarDriver() 
  : Node("star_driver")
  {
    this->declare_parameter("side", 4.0);  
    side_ = this->get_parameter("side").as_double();

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10, std::bind(&StarDriver::pose_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Menggambar bintang, sisi = %.2f", side_);
  }

private:
  enum class State { INIT, MOVE, TURN, DONE };

  static double normalize(double a) {return std::atan2(std::sin(a), std::cos(a));}

  void pose_callback(const turtlesim::msg::Pose & pose)
  {
    geometry_msgs::msg::Twist cmd; 

    switch (state_) {
      case State::INIT:
        heading_ = pose.theta;  
        start_x_ = pose.x;
        start_y_ = pose.y;
        state_ = State::MOVE;
        break;

      case State::MOVE: {
          const double traveled = std::hypot(pose.x - start_x_, pose.y - start_y_);
          if (traveled >= side_) {
            heading_ = normalize(heading_ + kTurnAngle);  // target arah berikutnya
            state_ = State::TURN;
          } else {
            cmd.linear.x = std::clamp(2.0 * (side_ - traveled), 0.3, 2.0);
          }
          break;
        }

      case State::TURN: {
          const double error = normalize(heading_ - pose.theta);
          if (std::fabs(error) < 0.005) {
            ++edges_done_;
            if (edges_done_ >= 5) {
              state_ = State::DONE;
              RCLCPP_INFO(this->get_logger(), "Bintang selesai!");
            } else {
              start_x_ = pose.x;
              start_y_ = pose.y;
              state_ = State::MOVE;
            }
          } else {
            cmd.angular.z = 6.0 * error;
          }
          break;
        }

      case State::DONE:
        break;
    }
    cmd_pub_->publish(cmd);
  }

  static constexpr double kTurnAngle = 4.0 * M_PI / 5.0;

  State state_ = State::INIT;
  int edges_done_ = 0;
  double side_;
  double heading_ = 0.0;
  double start_x_ = 0.0;
  double start_y_ = 0.0;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StarDriver>());
  rclcpp::shutdown();
  return 0;
}