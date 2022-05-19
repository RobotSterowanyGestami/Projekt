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




