# cpp_single_hop

A minimal ROS 2 C++ offboard controller. It arms one PX4 vehicle, takes off, moves to a nearby setpoint, and lands.

The primary simulation path uses `swarm_naive` for both one and multiple vehicles. This package remains as a small reference for the PX4 command and setpoint sequence.
