#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher(rclcpp::NodeOptions options)
    : Node("minimal_publisher", options), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::Int8>("motor_speed", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
      subscriber_ = this->create_subscription<std_msgs::msg::String>("topic",
      10, std::bind(&MinimalPublisher::topic_callback, this, std::placeholders::_1));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::Int8();
      message.data = 10;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.data);
      publisher_->publish(message);
    }
    void topic_callback(const std_msgs::msg::String &msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr publisher_;
    size_t count_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
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
