#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "control_toolbox/pid.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std::chrono_literals;

class PID_Node : public rclcpp::Node
{
  public:
    PID_Node(rclcpp::NodeOptions options)
    : Node("pid", options)
    {
      left_motor_publisher = this->create_publisher<std_msgs::msg::Float32>("post_pid_left_motor_speed", 10);
      right_motor_publisher = this->create_publisher<std_msgs::msg::Float32>("post_pid_right_motor_speed", 10);
      timer = this->create_wall_timer(
      10ms, std::bind(&PID_Node::timer_callback, this));
      motor_speed_subscriber = this->create_subscription<std_msgs::msg::Float32>("motor_speed",
      10, std::bind(&PID_Node::velocity_callback, this, std::placeholders::_1));      
      left_motor_speed_subscriber = this->create_subscription<std_msgs::msg::Float32>("left_encoder",
      10, std::bind(&PID_Node::left_motor_callback, this, std::placeholders::_1));
      right_motor_speed_subscriber = this->create_subscription<std_msgs::msg::Float32>("right_encoder",
      10, std::bind(&PID_Node::right_motor_callback, this, std::placeholders::_1));
      pid_controller = std::make_shared<control_toolbox::Pid>();
      pid_controller->initPid(0.6,0.0, 0.0, 0.08,-0.08, false);
      last_cmd_time = this->now();

    }

  private:
    void timer_callback()
    {
      rclcpp::Duration dt = this->now() - last_cmd_time;
      double error_left = desired_motor_speed - actual_left_motor_speed;
      float left_command = static_cast<float>(pid_controller->computeCommand(error_left, dt.nanoseconds()));
      auto message = std_msgs::msg::Float32();
      if(left_command > 0.22)
      {
              message.data = 0.22;
      }
      else if (left_command < -0.22)
      {
        message.data = -0.22;
      }
      else
      {
        message.data = left_command;
      }
      left_motor_publisher->publish(message);

      double error_right = desired_motor_speed - actual_right_motor_speed;
      float right_command = static_cast<float>(pid_controller->computeCommand(error_right, dt.nanoseconds()));
      auto r_message = std_msgs::msg::Float32();
      if(right_command > 0.22)
      {
        r_message.data = 0.22;
      }
      else if (right_command < -0.22)
      {
        r_message.data = -0.22;
      }
      else{
        r_message.data = right_command;
      }
      right_motor_publisher->publish(r_message);
    }
    void velocity_callback(const std_msgs::msg::Float32 &msg) 
    {
        desired_motor_speed = msg.data;
      }
    void left_motor_callback(const std_msgs::msg::Float32 &msg) 
    {
      actual_left_motor_speed = msg.data;
    }
    void right_motor_callback(const std_msgs::msg::Float32 &msg) 
    {
      actual_right_motor_speed = msg.data;
    }
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_motor_publisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_motor_publisher;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr motor_speed_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr left_motor_speed_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr right_motor_speed_subscriber;
    float desired_motor_speed;
    float actual_left_motor_speed = 0.0;
    float actual_right_motor_speed = 0.0;
    std::shared_ptr<control_toolbox::Pid> pid_controller;
    rclcpp::Time last_cmd_time;
    
};


int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::executors::SingleThreadedExecutor exec;
auto example_node = std::make_shared<PID_Node>(options);
  // control_toolbox::PidROS pid_ctrl(example_node);
  exec.add_node(example_node);
  while(rclcpp::ok())
    {
      exec.spin_once();
    }
  
  rclcpp::shutdown();
  return 0;
}


