# Simulation limits

The container builds PX4 SITL from stable release `v1.17.0` and uses Gazebo Harmonic with PX4's stock X500 simulation model. The flight software communicates with PX4 through ROS 2 Jazzy, `px4_msgs` from the matching `v1.17.0` tag, and Micro XRCE-DDS. QGroundControl uses MAVLink over UDP.

The simulated control path is:

```text
ROS 2 flight node -> uXRCE-DDS Agent -> PX4 SITL -> Gazebo model
QGroundControl <---------------------- MAVLink ------------------>
```

The workflow does not build or flash Pixhawk firmware. It does not model separate companion-computer hardware or radio links. Flight-network delay and loss require separate testing.

The current flight nodes do not detect obstacles or change routes around them. `swarm_comms` shares vehicle state but does not assign search areas or recover work after a vehicle failure.

SITL results do not replace flight testing. Validate failsafes, radio links, companion-computer timing, sensors, and airframe behavior on hardware before field deployment.
