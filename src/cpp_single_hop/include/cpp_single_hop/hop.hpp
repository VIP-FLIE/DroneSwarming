// ===========================================================================
// hop.hpp -- the Hop node: one drone, takes off, flies forward, lands.
//
// This file holds the declarations: the tunables, the shared helpers, and the
// Hop class. The method bodies are in hop.cpp, and main() is in main.cpp.
//
// Run it with the simulator already up (see docs/HOW-IT-WORKS.md):
//
//     ros2 run cpp_single_hop hop
//     ros2 run cpp_single_hop hop --ros-args -p altitude_m:=5.0 -p forward_m:=6.0
//
// WHAT THIS PROGRAM IS ACTUALLY DOING
// -----------------------------------
// It is not flying the drone. PX4 is flying the drone. This program is a
// backseat navigator: ten times a second it tells PX4 "I would like to be
// HERE", and PX4's position controller works out the motor commands that get
// there. That division never changes, in simulation or on the aircraft, and it
// is why a bug in here makes the drone go to the wrong place rather than fall
// out of the sky.
//
// The conversation with PX4 has exactly three rules, and essentially every
// "my offboard node does nothing" bug comes from breaking one of them:
//
//   1. QoS has to match, or your subscriptions silently never fire.
//   2. PX4 will not enter offboard mode until a setpoint stream is ALREADY
//      running, and drops out if it stops for about half a second.
//   3. PX4 will not arm until its preflight checks pass, which includes
//      having the ground control station link its failsafe depends on.
//
// Each rule is marked RULE 1/2/3 at the place it is obeyed.
// ===========================================================================
#pragma once

#include <cstdint>
#include <optional>
#include <string>

// Message definitions generated from PX4's own uORB message set. These are the
// exact structs PX4 publishes internally, so there is no translation layer and
// no place for a field to be misinterpreted.
//   https://github.com/PX4/px4_msgs/tree/release/1.16/msg
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>

namespace cpp_single_hop
{

using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;

// ---------------------------------------------------------------------------
// Tunables. Every number the program uses has a name here, so there is never a
// bare constant in the logic for you to have to decode.
// ---------------------------------------------------------------------------

/// How often we talk to PX4. RULE 2: PX4 drops out of offboard if the setpoint
/// stream pauses for roughly half a second, so anything at or above ~5 Hz is
/// safe. 10 Hz is the figure PX4's own examples use.
constexpr double CONTROL_LOOP_HZ = 10.0;

/// RULE 2 again: how much setpoint stream to build up before asking to arm.
/// Ten ticks at 10 Hz is one second, which is plenty for PX4 to notice.
constexpr int SETPOINTS_BEFORE_ARMING = 10;

/// How long to keep asking PX4 to arm before giving up. Generous, because a
/// freshly booted vehicle is still settling its position estimate and waiting
/// on its datalink, so early refusals are normal and temporary.
constexpr double ARMING_TIMEOUT_S = 60.0;

/// Repeat the arm request at 2 Hz rather than on every tick. Asking faster does
/// not make PX4 change its mind, it just floods the command link.
constexpr int ARMING_ATTEMPT_INTERVAL_TICKS = 5;

/// Stay quiet about arming for this long. A healthy vehicle arms well inside
/// it, so the only vehicle that says anything is one that is actually stuck.
constexpr double ARMING_QUIET_PERIOD_S = 3.0;

/// How close to a target counts as having arrived. A multicopter holding
/// position drifts by a few tens of centimetres, so demanding better than this
/// means never arriving at all.
constexpr double ARRIVAL_RADIUS_M = 0.5;

/// Throttle for repeated complaints, so a stuck vehicle prints every five
/// seconds instead of ten times a second.
constexpr int PROBLEM_REPORT_INTERVAL_MS = 5000;

/// PX4 stamps every message in microseconds since boot. ROS time counts
/// nanoseconds. This is the conversion, and forgetting it is a classic bug:
/// PX4 silently ignores setpoints whose timestamps look wrong.
constexpr int64_t NANOSECONDS_PER_MICROSECOND = 1000;

/// Parameters of VEHICLE_CMD_DO_SET_MODE. param1 is the MAVLink base mode,
/// where 1 means "the real mode is the custom mode in param2"; param2 is PX4's
/// custom main mode, where 6 is offboard.
///   https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_MODE
constexpr float BASE_MODE_CUSTOM_ENABLED = 1.0F;
constexpr float CUSTOM_MAIN_MODE_OFFBOARD = 6.0F;

/// A PX4 flight controller is always component 1 of its MAVLink system.
constexpr uint8_t PX4_COMPONENT_ID = 1;

// Flight shape. Change these two numbers and the drone flies somewhere else.
constexpr double DEFAULT_ALTITUDE_M = 3.0;
constexpr double DEFAULT_FORWARD_M = 3.0;

/// MAVLink system id. PX4 ignores any command addressed to a different system,
/// so this has to match the vehicle. SITL instance N gets system id N + 1, and
/// the default single-vehicle instance is 0, hence 1.
constexpr int DEFAULT_SYSTEM_ID = 1;

// ---------------------------------------------------------------------------
// RULE 1: QoS.
//
// This is the single most common reason a PX4 node appears to work while
// receiving nothing at all. PX4 publishes its /fmu/out/ topics as BEST_EFFORT
// with TRANSIENT_LOCAL durability. A default ROS 2 subscriber is RELIABLE and
// VOLATILE. Those are incompatible, so your subscription is created without an
// error, never matches the publisher, and your callback simply never fires.
// Nothing is logged. Nothing looks wrong.
//
// What makes it genuinely nasty: `ros2 topic echo` negotiates QoS and prints
// data happily, so the topic looks healthy from the command line while your
// node sits there receiving zero messages.
//
//   https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html
// ---------------------------------------------------------------------------

/// QoS for PX4 telemetry (/fmu/out/) and for the setpoints we stream to it.
///
/// Depth 1, because only the newest sample matters: an old position estimate is
/// worse than no position estimate.
rclcpp::QoS px4_stream_qos();

/// QoS for /fmu/in/vehicle_command, which is the one exception.
///
/// PX4's uXRCE-DDS client serves vehicle_command as a DDS replier rather than a
/// plain subscriber, and that replier is RELIABLE. A BEST_EFFORT publisher is
/// incompatible with it, so arm and mode commands would be dropped in silence.
rclcpp::QoS px4_command_qos();

/// Turn a PX4 namespace into a topic prefix: "px4_1" becomes "/px4_1".
///
/// An empty namespace gives an empty prefix, so topics are bare /fmu/... That
/// is what a real aircraft's companion computer sees and what SITL instance 0
/// publishes, which is why the same binary serves both with no special case.
/// You will need this the day you fly a second drone.
std::string px4_topic_prefix(const std::string & px4_namespace);

/// A position in the vehicle's local NED frame.
///
/// NED means North, East, Down, and the origin is wherever this vehicle was
/// when its position estimate started, i.e. its own takeoff point. DOWN IS
/// POSITIVE DOWNWARD, so three metres up is a down value of -3. Getting this
/// backwards is the other classic beginner bug, and it drives the drone into
/// the ground rather than into the air.
struct Position
{
  double north_m;
  double east_m;
  double down_m;
};

/// The stages of the flight, in the order they happen.
///
/// The first three are not a design choice, they are PX4's required order:
/// stream setpoints, then arm, then switch to offboard. The rest is the
/// actual flight.
enum class Phase
{
  Priming,           ///< building up the setpoint stream PX4 insists on
  Arming,            ///< asking PX4 to arm, repeatedly
  EngagingOffboard,  ///< asking PX4 to hand control to our setpoints
  Climbing,          ///< straight up to altitude_m_
  FlyingForward,     ///< forward_m_ metres north, holding altitude
  Landing,           ///< PX4's own landing routine has the controls
  Done               ///< on the ground, disarmed, nothing left to do
};

// ===========================================================================
// The node.
// ===========================================================================

class Hop : public rclcpp::Node
{
public:
  Hop();

private:
  // -- The control loop ---------------------------------------------------

  /// Runs CONTROL_LOOP_HZ times a second, for the whole flight.
  void run_control_tick();

  /// Where the drone should be right now.
  Position target_position() const;

  /// The state machine: decides when one phase is finished and the next begins.
  void advance_phase();

  /// RULE 3: get the vehicle armed, and explain it if PX4 says no.
  void run_arming();

  /// Turns PX4's own view of why it will not arm into something you can act on.
  std::string describe_arming_blocker() const;

  /// Say so plainly when a failsafe pulls the vehicle out of offboard mode.
  void warn_if_px4_took_control() const;

  // -- Reading the vehicle ------------------------------------------------

  bool is_armed() const;
  bool is_offboard() const;

  /// Straight-line distance from where PX4 says we are to where we asked to be.
  bool has_arrived() const;

  // -- Talking to PX4 -----------------------------------------------------

  uint64_t px4_timestamp_us() const;

  /// RULE 2, the stream itself.
  void publish_offboard_control_mode();

  /// The actual "please be here" message, in metres, in the local NED frame.
  void publish_setpoint(const Position & target);

  /// One MAVLink-style order to this vehicle's flight controller.
  void send_vehicle_command(uint32_t command, float param1 = 0.0F, float param2 = 0.0F);

  // -- State --------------------------------------------------------------

  double altitude_m_{DEFAULT_ALTITUDE_M};
  double forward_m_{DEFAULT_FORWARD_M};
  uint8_t system_id_{DEFAULT_SYSTEM_ID};

  Phase phase_{Phase::Priming};
  int priming_ticks_{0};
  int arming_attempts_{0};
  rclcpp::Time arming_started_;

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

}  // namespace cpp_single_hop
