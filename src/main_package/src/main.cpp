#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <poll.h>
#include <stdio.h>
//#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;
using namespace std_msgs::msg;
using namespace std;
using namespace std::placeholders;
using namespace rclcpp;

template <typename T>
class My_publisher{
  public:
 	My_publisher( Node* n, char topic[] ){
 		pub = n->create_publisher<T>(topic,10);
 	}
 	
 	void publish(T msg){
 		pub->publish(msg);
 	}
 	
 	
  private:
  	typename Publisher<T>::SharedPtr pub;
};

template <typename T>
class Subscriber{
  public:
 	Subscriber( Node* n, char topic[] ){
 		sub = n->create_subscription<T>(
			topic,
		 	10,
			bind(&Subscriber::callback, this, _1)
		);
 	}
 	
 	void callback(const T &msg){
 		this->msg.data = msg.data;
 	}
 	
 	T get_msg(){
 		return msg;
 	}
 	
 	
  private:
  	typename Subscription<T>::SharedPtr sub;
  	T msg;
};

class Main : public Node
{
  public:
    Main(NodeOptions options)
    : Node("main", options), count_(0)
    {
    	motor = new My_publisher<Int8>(this,"motor_speed");
    	servo = new My_publisher<Int16>(this,"servo_angle");
    	
    	motor_speed.data = 0;
    	servo_angle.data = 90;
    	
    	left_encoder = new Subscriber<Float32>(this,"left_encoder");
		 	 
		timer = this->create_wall_timer(
      		500ms,
      		bind(&Main::timer_callback, this)
      		);
    }
	
  private:
  	// Loop
    void timer_callback()
    {
   		struct pollfd mypoll = { 0, POLLIN }; 
   		
    	if( poll(&mypoll,1,0) ){
			
   			int d1,d2;
			
			int tmp = scanf("%d,%d",&d1,&d2);
			// printf("%d\n",tmp);
			
			switch(tmp){
				case 0:
					scanf(",%d",&d2 );
					servo_angle.data = d2;
					break;
				case 2:
					servo_angle.data = d2;
				case 1:
					motor_speed.data = d1;
			}
			
    		scanf("%*[^\n]");
    		
    		motor->publish(Int8(motor_speed));
    		servo->publish(Int16(servo_angle));
    	}
    }
    
    size_t count_;
    
	Int8 motor_speed;
	Int16 servo_angle;
    
    TimerBase::SharedPtr timer;
    
    My_publisher<Int8>* motor;
    My_publisher<Int16>* servo;
    Subscriber<Float32>* left_encoder;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::executors::SingleThreadedExecutor exec;
  
  auto example_node = std::make_shared<Main>(options);
  exec.add_node(example_node);
  while(rclcpp::ok())
    {
      exec.spin_once();
    }
  
  rclcpp::shutdown();
  return 0;
}
