# LAUNCH MOBILE ROBOT 
``` ros2 launch drive_to_point launch_all.launch.py ```
change pid gains in src/control/src/pid.cpp

# SET PID GAINS
Use plotjuggler: `sudo apt install ros-galactic-plotjuggler-ros`
In plotjuggler select Streaming: ROS2 Subscriber/ROS Topic Subscriber
Click `Start` - then select these topics in the pop-up window:
/motor_speed
/post_pid_*_motor_speed (left + right)
/left_encoder
/right_encoder

Motor speed is set value, encoder value is real value, PID is fed difference between those two. 
PID output is on /post_pid topics. 

In order to see plotjuggler output plotted:
in the bottom left topic list - click and drag topic name to the plotting area - output should start being plotted. 

Choose gains that don't oscilate but is still fast when set value is changed.


# NEW PACKAGE + NODE IN C++:
``` ros2 pkg create --build-type ament_cmake --node-name <node_name> <package_name> ```

warning: no `-` in node/package names, only `_`

# BOFORE ANYTHING ELSE

usage: 

``` source /opt/ros/galactic/setup.bash```

in `home/.../Raspberry`:

``` colcon build```

``` source install/setup.bash```

# DC DRIVER NODE

```ros2 run hal_dc_motors motor_driver```

DC Motor Driver is controlled by topic /motor_speed

# SERVO NODE

usage:

``` ros2 run hal_servo hal_servo_driver ```

# ULTRASOUND SENSORS NODE

``` ros2 run hal_ultrasound_sensor hal_ultrasound_sensor_driver ```

# ENCODERS

``` ros2 run hal_encoders hal_encoder_driver ```

# NETWORKING

conenct to wifi via terminal: ```sudo nmcli dev wifi connect <network-ssid> password <network-password>```

ssh: ```ssh projekt@<ip> -X```




