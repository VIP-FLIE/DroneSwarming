# Simulation limits

The container builds PX4 SITL v1.17.0 with Gazebo Harmonic and the stock X500 model. ROS 2 Jazzy communicates with PX4 through the matching `px4_msgs` package and Micro XRCE-DDS. QGroundControl uses MAVLink over UDP.

The control path is:

```text
mission_execution -> vehicle_control -> uXRCE-DDS Agent -> PX4 SITL -> Gazebo
QGroundControl <------------------------------ MAVLink -------------------->
```

The workflow does not build or flash Pixhawk firmware. It does not model companion-computer hardware, air-to-ground radio links, or physical sensor behavior. Flight-network delay and loss require separate testing.

The initial application has no obstacle detection, collision avoidance, peer discovery, search allocation, or failure recovery. `swarm_coordination` is a package scaffold.

SITL results do not establish hardware readiness. Hardware testing requires separate validation of failsafes, radio links, sensors, estimator behavior, and airframe response.
