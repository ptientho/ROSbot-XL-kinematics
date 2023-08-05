#include "armadillo"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/utilities.hpp"
#include "std_msgs/msg/detail/float32_multi_array__struct.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <functional>
#include <math.h>
#include <memory>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
using namespace arma;

class EightTrajectory : public rclcpp::Node {

public:
  EightTrajectory(std::string &node_name) : Node(node_name) {

    rcutils_logging_set_logger_level(this->get_logger().get_name(),
                                     RCUTILS_LOG_SEVERITY_INFO);

    WHEEL_BASE_DISTANCE = 0.170;
    TRACK_WIDTH = 0.269;
    WHEEL_RADIUS = 0.050;

    TF = {{-0.5 * WHEEL_BASE_DISTANCE - 0.5 * TRACK_WIDTH, 1, -1},
          {0.5 * WHEEL_BASE_DISTANCE + 0.5 * TRACK_WIDTH, 1, 1},
          {0.5 * WHEEL_BASE_DISTANCE + 0.5 * TRACK_WIDTH, 1, -1},
          {-0.5 * WHEEL_BASE_DISTANCE - 0.5 * TRACK_WIDTH, 1, 1}};

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&EightTrajectory::odom_callback, this,
                  std::placeholders::_1));

    wheel_vel_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/wheel_speed", 10);

    RCLCPP_INFO(this->get_logger(), "EightTrajectory class initialized.");
  }

  Col<double> vel_to_twist(const std::vector<double> &waypoint) {

    Mat<double> TF = {
        {1, 0, 0}, {0, cos(phi), sin(phi)}, {0, -sin(phi), cos(phi)}};

    Col<double> vel(waypoint.data(), waypoint.size());
    Col<double> twist = TF * vel;
    return twist;
  }

  void publish_wheel_vel(const Col<double> &twist) {

    Col<double> wheel_vel = TF * twist;

    wheel_vel_msg.data = {(float)wheel_vel(0), (float)wheel_vel(1),
                          (float)wheel_vel(2), (float)wheel_vel(3)};

    wheel_vel_pub_->publish(wheel_vel_msg);

  }
  void goto_waypoints(const std::vector<std::vector<double>> &traj) {

    int iteration = 300; //traverse with rate 0.01s for 3 seconds
    // loop through each waypoint
    for (const auto &w : traj) {
      for (int i = 1; i <= iteration; i++) {
        Col<double> tw = vel_to_twist(w);
        publish_wheel_vel(tw);
        std::this_thread::sleep_for(0.01s);
      }
    }
    RCLCPP_INFO(this->get_logger(),"Complete trajectory. Stopping robot.");
    wheel_vel_msg.data = {0,0,0,0};
    wheel_vel_pub_->publish(wheel_vel_msg);

  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_vel_pub_;
  double phi;

  float WHEEL_BASE_DISTANCE;
  float TRACK_WIDTH;
  float WHEEL_RADIUS;
  mat TF;
  std_msgs::msg::Float32MultiArray wheel_vel_msg;

  void odom_callback(const nav_msgs::msg::Odometry &msg) {

    // get phi real time
    tf2::Quaternion orientation(
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);

    tf2::Matrix3x3 mat(orientation);
    double r;
    double p;
    mat.getRPY(r, p, phi);
  }
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  std::string node_name = "eight_trajectory_node";
  auto eight_trajectory_node = std::make_shared<EightTrajectory>(node_name);

  std::vector<double> w1 = {0, 1, -1};
  std::vector<double> w2 = {0, 1, 1};
  std::vector<double> w3 = {0, 1, 1};
  std::vector<double> w4 = {1.5708, 1, -1};
  std::vector<double> w5 = {-3.1415, -1, -1};
  std::vector<double> w6 = {0, -1, 1};
  std::vector<double> w7 = {0, -1, 1};
  std::vector<double> w8 = {0, -1, -1};

  std::vector<std::vector<double>> trajectory = {w1, w2, w3, w4,
                                                 w5, w6, w7, w8};
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(eight_trajectory_node);

  RCLCPP_INFO(eight_trajectory_node->get_logger(), "Moving to waypoints");
  eight_trajectory_node->goto_waypoints(trajectory);

  executor.spin_node_once(eight_trajectory_node);

  rclcpp::shutdown();
  return 0;
}