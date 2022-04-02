# NEW PACKAGE + NODE:
``` ros2 pkg create --build-type ament_cmake --node-name <node_name> <package_name> ```

warning: no `-` in node/package names, only `_`


# DC DRIVER NODE
usage: 

``` source /opt/ros/galactic/setup.bash```

in `home/.../Raspberry`:

``` colcon build```

``` . install/setup.bash```

```ros2 run hal_dc_motors motor_driver```


DC Motor Driver is controlled by topic /motor_speed

