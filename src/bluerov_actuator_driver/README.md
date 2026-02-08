# cse_actuator_driver
ROS2 lifecycle node for the PCA9685 PWM/Servo driver on the CS Enterprise.

## Build instructions
1. On the Raspberry Pi, ensure that `wiringPi`, `i2c-dev` and `Eigen3` are installed
```
sudo apt install wiringpi
sudo apt install libi2c-dev
sudo apt install libeigen3-dev
```

2. Build from workspace with ``catkin_make``


<!-- ## Running the nodelet
```
rosrun nodelet nodelet manager __name:=nodelet_manager

rosrun nodelet nodelet load cse_actuator_driver/cse_actuator_driver_nodelet nodelet_manager __name:=cse_actuator_driver
``` -->

## Process real-time settings

These are the options that allows to configure the process real-time settings:
* `priority`: changes the process priority and set a real-time FIFO priority. This will set the
 priority of the thread where the ROS 2 executor is running.
* `cpu-affinity`: binds the application to a specific CPU core by setting a CPU mask. For
 example, to bind the process or thread to CPU 3 (or 2 starting from 0) we set the CPU mask to 4
  (100 in binary).
* `lock-memory`: pre-faults memory until no more page faults are seen. This usually allocated a 
 high amount of memory so make sure there is enough memory in the system.
* `lock-memory-size`: specifies the amount of memory we want to pre-allocate. For example 
 `lock-memory-size 100` pre-allocates 100 MB.
* `config-child-threads`: specifies if the RMW middleware child threads will inherit the
 main process settings. This applies for `priority` and `cpu-affinity options.`. For example, if 
 `config-child-threads False` is set, only the main thread where the ROS executor is set with the 
 `priority` and `cpu-affinity` options. If, `config-child-threads True` is set, the DDS threads
  will also inherit the same priority and CPU affinity configuration than the main thread.


## Parameters
Parameters can be configured in the `params.yaml` file in `./params/params.yaml`.

Example using the executable command line arguments (from your workspace):

```bash
ros2 run cse_actuator_driver cse_actuator_driver cse_actuator_driver_exe --ros-args --params-file ./src/cse_actuator_driver/params/params.yaml --priority 80 --cpu-affinity:=4 --lock-memory-size 100 --config-child-threads True 
```

## Topics
The node subscribes to the joystick and control input topics `/joy` and `/CSEI/u`, respectively.

## Usage
 Pressing `circle` on the connected joystick disables all actuators, while `cross` converts the input vector
<!-- $$u = (u_{\text{BT}}, u_{\text{VSP1}}, u_{\text{VSP2}}, \alpha_{\text{VSP1}}, \alpha_{\text{VSP2}}),$$ -->

![formula](https://latex.codecogs.com/svg.latex?\color{red}{%20u%20=%20(u_{\text{BT}},%20u_{\text{VSP1}},%20u_{\text{VSP2}},%20\alpha_{\text{VSP1}},%20\alpha_{\text{VSP2}}),})

from `/CSEI/u` into appropriate pwm signals.
