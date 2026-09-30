// ===========================================================================
// waypoint_drone.hpp -- one drone in a swarm that does not know it is a swarm.
//
// Run N copies of this program, one per vehicle, and you have a swarm. There
// is no central brain and no drone-to-drone traffic at all: every copy is
// handed a slot number and a flight plan at launch, and then flies it open
// loop. That is the whole idea of this package.
//
//     ros2 launch swarm_naive swarm.launch.py
//     ros2 launch swarm_naive swarm.launch.py count:=5 radius_m:=12.0
//
// The flight: each drone climbs off its own pad, flies once round a shared
// ring, comes back over its pad and lands. The ring has one vertex per drone
// and each drone enters at its own vertex, so they chase each other round it.
//
// WHAT THIS ADDS TO cpp_single_hop/hop.cpp
// -------------------------------------
// hop.cpp is one drone and is the file to read first. It covers the three
// rules PX4 imposes on any offboard program, and this file obeys the same
// three in the same places:
//
//   RULE 1  QoS has to match, or your subscriptions silently never fire.
//   RULE 2  PX4 needs a setpoint stream running BEFORE it will go offboard,
//           and drops out if the stream pauses for about half a second.
//   RULE 3  PX4 will not arm until its preflight checks pass, which includes
//           having the ground control station link its failsafe depends on.
//
// Everything past that is new, and it is all about there being more than one
// aircraft:
//
//   * THE SHARED FRAME. Every PX4 measures position from its own takeoff
//     point, so "north 8" is a different place for every drone. The ring is
//     therefore written in a shared frame that nobody's autopilot knows about,
//     and converted per drone in to_local(). See SwarmPoint vs LocalPoint.
//   * THE SLOT. One number, passed at launch, that decides which vertex this
//     drone enters at and which altitude it flies. It is the only thing that
//     makes one copy of this program behave differently from the next.
//   * ALTITUDE LAYERING. Drone k flies at altitude + k * layer, so the paths
//     can cross in plan view without the aircraft meeting. Read the next
//     section before deciding that is a good idea.
//
// WHAT THIS DELIBERATELY DOES NOT DO
// ----------------------------------
// No drone here ever learns where another drone is. Consequences, stated
// plainly because they are the reason the second package exists:
//
//   * No collision avoidance. Nothing in this program would notice, or do
//     anything about it, if two aircraft converged. The separate altitude per
//     slot is the only thing keeping them apart, and it is a static guess made
//     before takeoff, not a reaction to anything.
//   * No shared clock. Each drone starts its lap whenever it personally
//     finished arming. A drone that took twenty seconds longer to arm is not
//     twenty seconds late, it is permanently in the wrong part of the ring.
//   * No formation. The drones hold their spacing only as long as they happen
//     to fly at the same speed. Nothing corrects a drift.
//
// Run the launch file with `layer_m:=0.0` to put every drone on one level and
// watch what "naive" actually costs you. swarm_comms is the same flight with
// the drones talking to each other, and docs/SWARM.md explains the difference.
// ===========================================================================
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

// Generated from PX4's own uORB message set, so these structs are byte for
// byte what PX4 publishes internally.
//   https://github.com/PX4/px4_msgs/tree/release/1.16/msg
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>

namespace swarm_naive
{

using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;

// ---------------------------------------------------------------------------
// Tunables. Every number the program uses has a name here, so there is never a
// bare constant in the logic below for you to have to decode.
// ---------------------------------------------------------------------------

/// How often we talk to PX4. RULE 2: PX4 drops out of offboard if the setpoint
/// stream pauses for roughly half a second, so anything at or above ~5 Hz is
/// safe. 10 Hz is the figure PX4's own examples use.
constexpr double CONTROL_LOOP_HZ = 10.0;

/// RULE 2 again: how much setpoint stream to build up before asking to arm.
/// Ten ticks at 10 Hz is one second, which is plenty for PX4 to notice.
constexpr int SETPOINTS_BEFORE_ARMING = 10;

/// How long to keep asking PX4 to arm before giving up. Longer than the single
/// drone case, because several PX4 instances sharing one machine take longer to
/// settle their position estimates and to be found by the ground station.
constexpr double ARMING_TIMEOUT_S = 120.0;

/// Repeat the arm request at 2 Hz rather than on every tick. Asking faster does
/// not make PX4 change its mind, it just floods the command link.
constexpr int ARMING_ATTEMPT_INTERVAL_TICKS = 5;

/// Stay quiet about arming for this long. A healthy vehicle arms well inside
/// it, so the only vehicle that says anything is one that is actually stuck.
constexpr double ARMING_QUIET_PERIOD_S = 5.0;

/// How close to a waypoint counts as having arrived. A multicopter holding
/// position drifts by a few tens of centimetres, so demanding better than this
/// means never arriving at all. Larger than hop.cpp's 0.5 m on purpose: these
/// drones are meant to keep moving through the ring rather than stop on each
/// vertex, and a wider radius makes the corners rounder and the flight smoother.
constexpr double ARRIVAL_RADIUS_M = 0.7;

/// Complain if one leg of the route takes longer than this. Almost always means
/// PX4 has quietly left offboard mode and is flying something of its own.
constexpr double LEG_STALL_WARNING_S = 30.0;

/// Do not bother steering the nose when the target is this close. Without a
/// deadband the drone spins on the spot as it arrives, because the bearing to a
/// point you are sitting on is meaningless.
constexpr double YAW_DEADBAND_M = 1.0;

/// Throttle for repeated complaints, so a stuck vehicle prints every five
/// seconds instead of ten times a second.
constexpr int PROBLEM_REPORT_INTERVAL_MS = 5000;

/// PX4 stamps every message in microseconds since boot. ROS time counts
/// nanoseconds. Forgetting this is a classic bug: PX4 silently ignores
/// setpoints whose timestamps look wrong.
constexpr int64_t NANOSECONDS_PER_MICROSECOND = 1000;

/// Parameters of VEHICLE_CMD_DO_SET_MODE. param1 is the MAVLink base mode,
/// where 1 means "the real mode is the custom mode in param2"; param2 is PX4's
/// custom main mode, where 6 is offboard.
///   https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_MODE
constexpr float BASE_MODE_CUSTOM_ENABLED = 1.0F;
constexpr float CUSTOM_MAIN_MODE_OFFBOARD = 6.0F;

/// A PX4 flight controller is always component 1 of its MAVLink system.
constexpr uint8_t PX4_COMPONENT_ID = 1;

/// Sanity ceiling on swarm size. Nothing here breaks above it, but drone-gcs
/// heartbeats SITL instances 0-10, so drone 11 would never be able to arm.
constexpr int MAX_SWARM_SIZE = 10;

// ---------------------------------------------------------------------------
// RULE 1: QoS.
//
// The single most common reason a PX4 node appears to work while receiving
// nothing. PX4 publishes its /fmu/out/ topics as BEST_EFFORT with
// TRANSIENT_LOCAL durability; a default ROS 2 subscriber is RELIABLE and
// VOLATILE. Those are incompatible, so the subscription is created without an
// error, never matches the publisher, and the callback simply never fires.
//
// `ros2 topic echo` negotiates QoS and prints data happily, so the topic looks
// healthy from the command line while your node receives zero messages.
//
//   https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html
// ---------------------------------------------------------------------------

/// QoS for PX4 telemetry (/fmu/out/) and for the setpoints we stream to it.
/// Depth 1, because only the newest sample matters: an old position estimate is
/// worse than no position estimate.
rclcpp::QoS px4_stream_qos();

/// QoS for /fmu/in/vehicle_command, which is the one exception. PX4's
/// uXRCE-DDS client serves vehicle_command as a DDS replier, and that replier
/// is RELIABLE. A BEST_EFFORT publisher is incompatible with it, so arm and
/// mode commands would be dropped in silence.
rclcpp::QoS px4_command_qos();

/// Turn a PX4 namespace into a topic prefix: "px4_1" becomes "/px4_1".
///
/// SITL instance 0 publishes on bare /fmu/..., exactly like a real aircraft's
/// autopilot, and instance N above 0 publishes on /px4_N/fmu/.... An empty
/// namespace therefore gives an empty prefix and the same binary serves both.
/// Verified in PX4 v1.16.1, ROMFS/px4fmu_common/init.d-posix/rcS line 299.
std::string px4_topic_prefix(const std::string & px4_namespace);

// ---------------------------------------------------------------------------
// THE TWO FRAMES.
//
// These hold the same three numbers and mean completely different things, so
// they are separate types rather than one shared one. Handing a SwarmPoint to
// something that wants a LocalPoint is then a compile error instead of a drone
// flying to the wrong field. This is the cheapest bug prevention in the file
// and it costs one extra struct.
// ---------------------------------------------------------------------------

/// A point in the SHARED SWARM FRAME: the frame the flight plan is written in.
///
/// Its origin is drone 0's pad, and no autopilot anywhere knows it exists. It
/// is a fiction agreed between the launch file and every node, which is exactly
/// what a formation needs and exactly what PX4 does not provide.
struct SwarmPoint
{
  double north_m;
  double east_m;
  double down_m;   ///< NED: down is POSITIVE DOWNWARD, so 5 m up is -5.
};

/// A point in THIS VEHICLE'S OWN local NED frame: the frame PX4 speaks.
///
/// Its origin is wherever this drone's position estimate started, which in
/// practice is its own pad. With one drone the distinction is invisible. With
/// several it is the central problem of swarming.
struct LocalPoint
{
  double north_m;
  double east_m;
  double down_m;
};

/// The stages of the flight, in the order they happen.
///
/// The first three are not a design choice, they are PX4's required order:
/// stream setpoints, then arm, then switch to offboard. Compare hop.cpp,
/// which spells the flight itself out as phases; here the flight is a list of
/// waypoints and an index, so it needs only one.
enum class Phase
{
  Priming,           ///< building up the setpoint stream PX4 insists on
  Arming,            ///< asking PX4 to arm, repeatedly
  EngagingOffboard,  ///< asking PX4 to hand control to our setpoints
  Flying,            ///< working down route_, one waypoint at a time
  Landing,           ///< PX4's own landing routine has the controls
  Done               ///< on the ground, disarmed, nothing left to do
};

// ===========================================================================
// The node. One instance of this flies one aircraft.
// ===========================================================================

class WaypointDrone : public rclcpp::Node
{
public:
  WaypointDrone();

private:
  // -- Setting up ---------------------------------------------------------

  /// Refuse to construct on a parameter that cannot mean anything.
  void validate_parameters() const;

  /// The altitude this drone flies at, which is the only vertical deconfliction
  /// in the whole package. Drone 0 gets altitude_m_, drone 1 gets one layer
  /// above it, and so on.
  double station_altitude_m() const;

  /// Vertex `index` of the ring, in the shared swarm frame.
  SwarmPoint ring_vertex(int index, double down_m) const;

  /// The whole flight, as a list of points in the shared swarm frame.
  std::vector<SwarmPoint> build_route() const;

  void announce_plan(const std::string & prefix) const;

  // -- The control loop ---------------------------------------------------

  /// Runs CONTROL_LOOP_HZ times a second, for the whole flight.
  void run_control_tick();

  /// Where this drone should be right now, in the shared swarm frame.
  SwarmPoint current_target() const;

  /// THE CONVERSION. This is the entire swarm problem, in two subtractions.
  LocalPoint to_local(const SwarmPoint & point) const;

  /// The state machine: decides when one phase is finished and the next begins.
  void advance_phase();

  /// RULE 3: get the vehicle armed, and explain it if PX4 says no.
  void run_arming();

  /// Work down the route, one waypoint at a time, and land off the end of it.
  void run_flying();

  /// A leg that never finishes is nearly always PX4 having left offboard mode,
  /// so say which waypoint is stuck rather than sitting there silently.
  void warn_if_leg_is_stalled() const;

  /// A failsafe pulls the vehicle out of offboard mode, and PX4 is right to win
  /// that argument. Say so plainly rather than streaming setpoints at a vehicle
  /// that has stopped listening.
  void warn_if_px4_took_control() const;

  /// Turns PX4's own view of why it will not arm into something you can act on.
  std::string describe_arming_blocker() const;

  // -- Reading the vehicle ------------------------------------------------

  bool is_armed() const;
  bool is_offboard() const;

  /// Straight-line distance from where PX4 says we are to a point in our own
  /// local frame. Both sides have to be in the same frame, which is what the
  /// LocalPoint type is there to enforce.
  double distance_to(const LocalPoint & target) const;

  bool has_arrived() const;

  // -- Talking to PX4 -----------------------------------------------------

  uint64_t px4_timestamp_us() const;

  /// RULE 2, the stream itself.
  void publish_offboard_control_mode();

  /// The actual "please be here" message, in metres, in THIS vehicle's local
  /// NED frame. By the time a point reaches here it has already been through
  /// to_local(), which is why the argument type is LocalPoint.
  void publish_setpoint(const LocalPoint & target);

  /// Point the nose along the direction of travel.
  double update_yaw_setpoint(const LocalPoint & target);

  /// One MAVLink-style order to this vehicle's flight controller.
  void send_vehicle_command(uint32_t command, float param1 = 0.0F, float param2 = 0.0F);

  // -- State --------------------------------------------------------------

  // Identity, fixed at launch.
  int slot_{0};
  int count_{3};
  uint8_t system_id_{1};
  SwarmPoint home_{0.0, 0.0, 0.0};

  // The shared flight plan, fixed at launch.
  double ring_centre_north_m_{10.0};
  double ring_centre_east_m_{0.0};
  double radius_m_{8.0};
  double altitude_m_{4.0};
  double layer_m_{2.0};
  int laps_{1};
  std::vector<SwarmPoint> route_;

  // Where we are in the flight.
  Phase phase_{Phase::Priming};
  std::size_t route_index_{0};
  int priming_ticks_{0};
  int arming_attempts_{0};
  rclcpp::Time arming_started_;
  rclcpp::Time leg_started_;
  std::optional<double> yaw_setpoint_rad_;

  // std::optional, not a pointer or a "have I got one yet" flag. Empty means
  // PX4 has not told us anything yet, and the type makes that impossible to
  // forget to check.
  std::optional<VehicleLocalPosition> local_position_;
  std::optional<VehicleStatus> vehicle_status_;

  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_mode_publisher_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr setpoint_publisher_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr command_publisher_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_subscription_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_subscription_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace swarm_naive
