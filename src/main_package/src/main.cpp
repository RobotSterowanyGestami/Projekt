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
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;
using namespace std_msgs::msg;
using namespace sensor_msgs::msg;
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
 		this->msg = msg;
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
    	
    	start = false;
    	
    	motor_speed.data = 0;
    	servo_angle.data = 90;
    	
    	left_encoder = new Subscriber<Float32>(this,"left_encoder");
    	imu = new Subscriber<Imu>(this,"hal_imu");
		 	 
		timer = this->create_wall_timer(
      		100ms,
      		bind(&Main::timer_callback, this)
      		);
    }
	
  private:
  	// Loop
    void timer_callback()
    {
    	float vx,vy,vz, acc;
    	vx = imu->get_msg().linear_acceleration.x;
    	vy = imu->get_msg().linear_acceleration.y;
    	vz = imu->get_msg().linear_acceleration.z;
    	
    	acc = vx*vx + vy*vy + vz*vz;
    	
    	vx = imu->get_msg().angular_velocity.x;
    	vy = imu->get_msg().angular_velocity.y;
    	vz = imu->get_msg().angular_velocity.z;
    	
    	float acu = 0.05;
    	
    	if( vx < acu and vx > -acu )
    		vx = 0;
    	if( vy < acu and vy > -acu )
    		vy = 0;
    	if( vz < acu and vz > -acu )
    		vz = 0;
    		
    	
    	x += vx; y += vy; z += vz;

    	//printf("acc: %0.3f\nx: %0.3f, y: %0.3f, z: %0.3f\n",acc,x,y,z);
    	if( acc > 1500 ){
    		start = true;
    		x = 0; y = 0; z = 0;
    		printf("-----------------\n");
    		printf("-----------------\n");
    	}
    	printf("x: %0.3f, y: %0.3f\n",x,y);
    	
    	
    	if( x > 13 )
    		motor_speed.data = -100;
    	if( x < 3 and x > -5 )
    		motor_speed.data = 0;
    	if(  x < -5 and x > -15  )
    		motor_speed.data = (-x-5)*7 + 30;
    	if( x < 13 and x > 3 )
    		motor_speed.data = -(x-3)*7 - 30;
    	if( x < -15 )
    		motor_speed.data = 100;
    		
    	servo_angle.data = -y*3+90;
    	
    	if( !start )
    	{
    		motor_speed.data = 0;
    		servo_angle.data = 90;
    	}
    	
    	printf("mot: %d, serv: %d\n",motor_speed.data,servo_angle.data);
    		    	
    	// 30 - 100
		motor->publish(Int8(motor_speed));

		// 45-135
		servo->publish(Int16(servo_angle));
    }
    
    size_t count_;
    
	Int8 motor_speed;
	Int16 servo_angle;
	
	bool start;
	float x,y,z;
    
    TimerBase::SharedPtr timer;
    
    My_publisher<Int8>* motor;
    My_publisher<Int16>* servo;
    
    Subscriber<Imu>* imu; 
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
