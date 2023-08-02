#include "armadillo"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/utilities.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <functional>
#include <iostream>
#include <ostream>
#include <vector>

using namespace std::chrono_literals;
using namespace arma;

class KinematicModel : public rclcpp::Node {

public:
  KinematicModel() : Node("kinematic_model") {
    rcutils_logging_set_logger_level(this->get_logger().get_name(),
                                     RCUTILS_LOG_SEVERITY_INFO);

    WHEEL_BASE_DISTANCE = 0.170;
    TRACK_WIDTH = 0.269;
    WHEEL_RADIUS = 0.050;

    TF = {{static_cast<float>(-0.5 * WHEEL_BASE_DISTANCE - 0.5 * TRACK_WIDTH),
           1, -1},
          {
              static_cast<float>(0.5 * WHEEL_BASE_DISTANCE + 0.5 * TRACK_WIDTH),
              1,
          },
          {static_cast<float>(0.5 * WHEEL_BASE_DISTANCE + 0.5 * TRACK_WIDTH), 1,
           -1},
          {static_cast<float>(-0.5 * WHEEL_BASE_DISTANCE - 0.5 * TRACK_WIDTH),
           1, 1}};

    inverseTF = pinv(TF);
    std::cout << "TF matrix:\n" << inverseTF << std::endl;
    // publish to /cmd_vel topic
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    // subscribe to /wheel_speed topic
    wheel_vel_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/wheel_speed", 10,
        std::bind(&KinematicModel::wheel_vel_callback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Constructor Initialized");
  }

private:
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_vel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  geometry_msgs::msg::Twist twist_msg;
  float WHEEL_BASE_DISTANCE; // 2L
  float TRACK_WIDTH;         // 2W
  float WHEEL_RADIUS;        // r
  fmat TF;
  fmat inverseTF;

  void wheel_vel_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {

    // get wheel velocities
    std::vector<float> wheel_vels = msg->data;
    Col<float> u(wheel_vels.data(), wheel_vels.size());
    std::cout << "U vector: \n" << u << std::endl;
    // calculate the dot product of wheel velocities and inverse of TF matrix
    Col<float> v = inverseTF * u;
    std::cout << "V vector: \n" << v << std::endl;
    float omega = v(0);
    float vx = v(1);
    float vy = v(2);

    // publish /cmd_vel
    twist_msg.linear.x = vx;
    twist_msg.linear.y = vy;
    twist_msg.angular.z = omega;
    twist_pub_->publish(twist_msg);
  }
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  auto kinematic_model_node = std::make_shared<KinematicModel>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(kinematic_model_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}