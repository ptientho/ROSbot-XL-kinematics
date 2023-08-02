#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/utilities.hpp"
#include "rcutils/logging.h"
#include "std_msgs/msg/detail/float32_multi_array__struct.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <memory>
#include <string>
#include <thread>

using VelArray = std_msgs::msg::Float32MultiArray;
using namespace std::chrono_literals;

class WheelVelocityPub : public rclcpp::Node{

public:
    WheelVelocityPub() : Node("wheel_velocities_publisher") {
    
        rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);

        //initialize publisher
        wheel_vel_pub_ = this->create_publisher<VelArray>("/wheel_speed", 10);
        RCLCPP_INFO(this->get_logger(),"Initialized wheel velocities publisher node");
        //call sequence movement
        timer_ = this->create_wall_timer(500ms, std::bind(&WheelVelocityPub::move_sequence,this));    
    
    }

    void move_forward(float vel){
    
        RCLCPP_INFO(this->get_logger(),"Move forward");
        wheel_vel_msg.data = {vel ,vel ,vel, vel};
        wheel_vel_pub_->publish(wheel_vel_msg);
    
    }

    void move_backward(float vel){
    
        RCLCPP_INFO(this->get_logger(),"Move backward");
        wheel_vel_msg.data = {-vel ,-vel ,-vel, -vel};
        wheel_vel_pub_->publish(wheel_vel_msg);
    
    }

    void move_sideway(const std::string &direction, float vel){
    
        if (direction == "left"){
            RCLCPP_INFO(this->get_logger(),"Move left");
            wheel_vel_msg.data = {-vel ,vel ,-vel, vel};
            wheel_vel_pub_->publish(wheel_vel_msg);
        
        } else if (direction == "right") {
            RCLCPP_INFO(this->get_logger(),"Move right");
            wheel_vel_msg.data = {vel ,-vel ,vel, -vel};
            wheel_vel_pub_->publish(wheel_vel_msg);
        }
        
 
    }

    void turn(const std::string &direction, float vel){
    
        if (direction == "cw"){
            RCLCPP_INFO(this->get_logger(),"Turn clockwise");
            wheel_vel_msg.data = {-vel ,vel ,vel, -vel};
            wheel_vel_pub_->publish(wheel_vel_msg);
        
        
        }else if(direction == "ccw"){
            RCLCPP_INFO(this->get_logger(),"Turn counter-clockwise");
            wheel_vel_msg.data = {vel ,-vel ,-vel, vel};
            wheel_vel_pub_->publish(wheel_vel_msg);
        
        }
    
    }

    void stop(){
    
        RCLCPP_INFO(this->get_logger(),"Stop");
        wheel_vel_msg.data = {0.0 ,0.0 ,0.0, 0.0};
            wheel_vel_pub_->publish(wheel_vel_msg);
    }

    void move_sequence(){
    
        //move forward
        this->move_forward(0.3);
        std::this_thread::sleep_for(3s);        
        //move backward
        this->move_backward(0.3);
        std::this_thread::sleep_for(3s);
        //move sideway to the left
        this->move_sideway("left", 0.3);
        std::this_thread::sleep_for(3s);
        //move sideway to the right
        this->move_sideway("right", 0.3);
        std::this_thread::sleep_for(3s);
        //turn cw
        this->turn("cw", 0.3);
        std::this_thread::sleep_for(3s);
        //turn ccw
        this->turn("ccw", 0.3);
        std::this_thread::sleep_for(3s);
        //stop
        this->stop();
    }

private:
    rclcpp::Publisher<VelArray>::SharedPtr wheel_vel_pub_;
    VelArray wheel_vel_msg;
    rclcpp::TimerBase::SharedPtr timer_;

};


int main(int argc, char** argv){

    rclcpp::init(argc, argv);
    auto wheel_velocities_node = std::make_shared<WheelVelocityPub>();
    
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(wheel_velocities_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;

}