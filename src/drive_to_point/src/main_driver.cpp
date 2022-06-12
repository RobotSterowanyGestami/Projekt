#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher(rclcpp::NodeOptions options)
    : Node("main_driver", options)
    {
      motor_publisher = this->create_publisher<std_msgs::msg::Float32>("motor_speed", 10);
      servo_publisher = this->create_publisher<std_msgs::msg::Int16>("servo_angle", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
      subscriber = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel",
      10, std::bind(&MinimalPublisher::topic_callback, this, std::placeholders::_1));
      linear_speed = 0;
      angular_speed = 0;
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::Float32();
      if(linear_speed >= 0.22)
      {
		  linear_speed = 0.22;
	  }
      else if(linear_speed <=-0.22)
     {
        linear_speed = -0.22;
     }
      message.data = linear_speed;
      motor_publisher->publish(message);
      auto servo_message = std_msgs::msg::Int16();
      if(angular_speed == 0)
      {
		  servo_message.data = 90;
	  }
	  else
	  {
		  
      servo_message.data = int16_t((-1)*(asin(x/(linear_speed/angular_speed)))*180/3.1415+90);
      }
      servo_publisher->publish(servo_message);
    }
    void topic_callback(const geometry_msgs::msg::Twist &msg)
    {
      linear_speed = msg.linear.x;
      angular_speed = msg.angular.z;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr motor_publisher;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr servo_publisher;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber;
    double linear_speed;
    double angular_speed;
    double x=0.19;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::executors::SingleThreadedExecutor exec;
  auto example_node = std::make_shared<MinimalPublisher>(options);
  exec.add_node(example_node);
  while(rclcpp::ok())
    {
      exec.spin_once();
    }
  
  rclcpp::shutdown();
  return 0;
}
