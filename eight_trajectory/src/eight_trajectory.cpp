#include "armadillo"
#include "mutex"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/utilities.hpp"
#include "rcutils/logging.h"
#include "std_msgs/msg/detail/float32_multi_array__struct.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>
#include <functional>
#include <limits>
#include <math.h>
#include <memory>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
using namespace arma;

class EightTrajectory : public rclcpp::Node {

public:
  EightTrajectory(const std::string &node_name,
                  const std::vector<std::vector<double>> &traj)
      : Node(node_name), trajectory(traj) {

    rcutils_logging_set_logger_level(this->get_logger().get_name(),
                                     RCUTILS_LOG_SEVERITY_INFO);
    current_waypoint_index = 0;
    absolute_waypoint =
        trajectory[current_waypoint_index]; // set the first waypoint to the
                                            // absolute frame

    WHEEL_BASE_DISTANCE = 0.170;
    TRACK_WIDTH = 0.269;
    WHEEL_RADIUS = 0.050;
    RHO = 1000;

    TF = {{-0.5 * WHEEL_BASE_DISTANCE - 0.5 * TRACK_WIDTH, 1.0, -1.0},
          {0.5 * WHEEL_BASE_DISTANCE + 0.5 * TRACK_WIDTH, 1.0, 1.0},
          {0.5 * WHEEL_BASE_DISTANCE + 0.5 * TRACK_WIDTH, 1.0, -1.0},
          {-0.5 * WHEEL_BASE_DISTANCE - 0.5 * TRACK_WIDTH, 1.0, 1.0}};

    call_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions opt;
    opt.callback_group = call_group_;

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 50,
        std::bind(&EightTrajectory::odom_callback, this, std::placeholders::_1),
        opt);

    wheel_vel_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/wheel_speed", 10);

    RCLCPP_INFO(this->get_logger(), "EightTrajectory class initialized.");

    timer_ = this->create_wall_timer(
        500ms, std::bind(&EightTrajectory::goto_waypoints, this), call_group_);
  }

  Col<double> get_twist(const std::vector<double> &waypoint) {
    std::lock_guard<std::mutex> lock(pose_mutex);
    double pos_x = x;
    double pos_y = y;
    double phi = theta;

    // RCLCPP_INFO(this->get_logger(),"Pos_x: %f",pos_x);
    mat TF_odom_robot(4, 4, arma::fill::zeros);
    mat TF_robot_goal(4, 4, arma::fill::zeros);

    TF_odom_robot(0, 0) = cos(phi);
    TF_odom_robot(0, 1) = sin(phi);
    TF_odom_robot(0, 3) = pos_x;
    TF_odom_robot(1, 0) = -sin(phi);
    TF_odom_robot(1, 1) = cos(phi);
    TF_odom_robot(1, 3) = pos_y;
    TF_odom_robot(2, 2) = 1.0;
    TF_odom_robot(3, 3) = 1.0;

    TF_robot_goal(0, 0) = cos(waypoint[0]);
    TF_robot_goal(0, 1) = sin(waypoint[0]);
    TF_robot_goal(0, 3) = waypoint[1];
    TF_robot_goal(1, 0) = -sin(waypoint[0]);
    TF_robot_goal(1, 1) = cos(waypoint[0]);
    TF_robot_goal(1, 3) = waypoint[2];
    TF_robot_goal(2, 2) = 1.0;
    TF_robot_goal(3, 3) = 1.0;

    // std::cout << "TF matrix:\n" << TF_odom_robot << std::endl;
    double delta_x = TF_robot_goal(0, 3) - TF_odom_robot(0, 3);
    // std::cout << "Dx:\n" << delta_x << std::endl;
    double delta_y = TF_robot_goal(1, 3) - TF_odom_robot(1, 3);
    // std::cout << "Dy:\n" << delta_y << std::endl;
    //  RCLCPP_DEBUG(this->get_logger(),"Dx: %f",delta_x);
    double rho = sqrt(pow(delta_x, 2) + pow(delta_y, 2));

    double alpha = -phi + atan2(delta_y, delta_x);
    double beta = -phi - alpha;

    std::vector<double> params = {rho, alpha, beta};
    RHO = rho;
    // RCLCPP_INFO(this->get_logger(), "Rho: %f, Alpha: %f", rho, alpha);
    Col<double> twist = calculate_twist(params);
    pose_mutex.unlock();
    return twist;
  }

  void goto_waypoints() {

    if (trajectory.empty()) {
      RCLCPP_WARN(this->get_logger(), "No waypoints in trajectory.");
      return;
    }

    // Start the iteration process
    iterate_through_waypoints();
  }

  void iterate_through_waypoints() {

    // RCLCPP_DEBUG(this->get_logger(), "Executing waypoint: {%f,%f,%f}",
    //              absolute_waypoint[0], absolute_waypoint[1],
    //              absolute_waypoint[2]);
    RCLCPP_INFO(this->get_logger(), "Executing waypoint #%d",
                current_waypoint_index + 1);
    Col<double> twist = get_twist(absolute_waypoint);
    publish_wheel_vel(twist);

    if (RHO <= 0.04) {
      // Move to the next waypoint
      current_waypoint_index++;
      if (current_waypoint_index >= trajectory.size()) {
        // All waypoints have been reached
        RCLCPP_INFO(this->get_logger(), "Complete trajectory. Stopping robot.");
        wheel_vel_msg.data = {0, 0, 0, 0};
        wheel_vel_pub_->publish(wheel_vel_msg);
        timer_->cancel();
        return;
      }

      const auto &current_waypoint = trajectory[current_waypoint_index];
      absolute_waypoint[0] += current_waypoint[0];
      absolute_waypoint[1] += current_waypoint[1];
      absolute_waypoint[2] += current_waypoint[2];

      // RCLCPP_INFO(this->get_logger(), "Completed waypoint #%d",
      //            current_waypoint_index);
    }
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr call_group_;
  std::mutex pose_mutex;

  int current_waypoint_index;
  std::vector<double> absolute_waypoint;

  double x;
  double y;
  double theta;

  float WHEEL_BASE_DISTANCE;
  float TRACK_WIDTH;
  float WHEEL_RADIUS;
  double RHO;
  mat TF;
  std_msgs::msg::Float32MultiArray wheel_vel_msg;
  std::vector<std::vector<double>> trajectory;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(pose_mutex);
    x = (double)msg->pose.pose.position.x;
    y = (double)msg->pose.pose.position.y;
    // get phi real time
    tf2::Quaternion orientation(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    tf2::Matrix3x3 mat(orientation);
    double r;
    double p;
    mat.getRPY(r, p, theta);
    // RCLCPP_INFO(this->get_logger(), "Pos_x: %f", y);
    pose_mutex.unlock();
  }

  Col<double> calculate_twist(const std::vector<double> &params) {

    const double K_rho = 0.3;
    const double K_alpha = 0.8;
    const double K_beta = -0.15;

    double linear_vel = K_rho * params[0];
    double angular_vel = K_alpha * params[1] + K_beta * params[2];

    std::vector<double> twist = {linear_vel, angular_vel};
    Col<double> twist_vec(twist.data(), twist.size());
    return twist_vec;
  }

  void publish_wheel_vel(const Col<double> &twist) {

    Col<double> wheel_vel = ((1 / WHEEL_RADIUS) * TF *
                             Col<double>{twist(1), twist(0), 0}); // w, vx, vy

    wheel_vel_msg.data = {(float)wheel_vel(0), (float)wheel_vel(1),
                          (float)wheel_vel(2), (float)wheel_vel(3)};

    // RCLCPP_INFO(this->get_logger(),
    //             "wheel_1: %f, wheel_2: %f, wheel_3: %f, wheel_4: %f",
    //             wheel_vel(0), wheel_vel(1), wheel_vel(2), wheel_vel(3));
    wheel_vel_pub_->publish(wheel_vel_msg);
  }
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  std::string node_name = "eight_trajectory_node";
  std::vector<double> w1 = {0, 1, -1};
  std::vector<double> w2 = {0, 1, 1};
  std::vector<double> w3 = {0, 1, 1};
  std::vector<double> w4 = {1.5708, 1, -1};
  std::vector<double> w5 = {-3.1415, -1, -1};
  std::vector<double> w6 = {0, -1, 1};
  std::vector<double> w7 = {0, -1, 1};
  std::vector<double> w8 = {0, -1, -1};

  // std::vector<std::vector<double>> trajectory = {w1, w2};
  std::vector<std::vector<double>> trajectory = {w1, w2, w3, w4,
                                                 w5, w6, w7, w8};
  auto eight_trajectory_node =
      std::make_shared<EightTrajectory>(node_name, trajectory);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(eight_trajectory_node);
  RCLCPP_INFO(eight_trajectory_node->get_logger(), "Moving to waypoints");
  executor.spin();

  rclcpp::shutdown();
  return 0;
}