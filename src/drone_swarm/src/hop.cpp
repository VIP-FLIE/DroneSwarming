// ===========================================================================
// hop.cpp -- the smallest useful PX4 flight program.
//
// One drone. Takes off, flies a few metres forward, lands. One file, no
// abstractions to chase, nothing here you will not need again.
//
// Run it with the simulator already up (see docs/HOW-IT-WORKS.md):
//
//     ros2 run drone_swarm hop
//     ros2 run drone_swarm hop --ros-args -p altitude_m:=5.0 -p forward_m:=6.0
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
// Each rule is marked RULE 1/2/3 at the place it is obeyed below.
// ===========================================================================

#include <cmath>
#include <cstdint>
#include <exception>
#include <memory>
#include <optional>
#include <stdexcept>
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

namespace drone_swarm
{
namespace
{

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
rclcpp::QoS px4_stream_qos()
{
  rclcpp::QoS qos{rclcpp::KeepLast(1)};
  qos.best_effort();
  qos.transient_local();
  return qos;
}

/// QoS for /fmu/in/vehicle_command, which is the one exception.
///
/// PX4's uXRCE-DDS client serves vehicle_command as a DDS replier rather than a
/// plain subscriber, and that replier is RELIABLE. A BEST_EFFORT publisher is
/// incompatible with it, so arm and mode commands would be dropped in silence.
rclcpp::QoS px4_command_qos()
{
  rclcpp::QoS qos{rclcpp::KeepLast(1)};
  qos.reliable();
  qos.transient_local();
  return qos;
}

/// Turn a PX4 namespace into a topic prefix: "px4_1" becomes "/px4_1".
///
/// An empty namespace gives an empty prefix, so topics are bare /fmu/... That
/// is what a real aircraft's companion computer sees and what SITL instance 0
/// publishes, which is why the same binary serves both with no special case.
/// You will need this the day you fly a second drone.
std::string px4_topic_prefix(const std::string & px4_namespace)
{
  const auto first = px4_namespace.find_first_not_of('/');
  if (first == std::string::npos) {
    return "";
  }
  const auto last = px4_namespace.find_last_not_of('/');
  return "/" + px4_namespace.substr(first, last - first + 1);
}

}  // namespace

using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;

// ===========================================================================
// The node.
// ===========================================================================

class Hop : public rclcpp::Node
{
public:
  Hop()
  : Node("hop")
  {
    // -- Parameters -------------------------------------------------------
    // Declared with defaults, overridable from the command line:
    //   ros2 run drone_swarm hop --ros-args -p altitude_m:=5.0
    const std::string px4_namespace = declare_parameter<std::string>("px4_ns", "");
    system_id_ = static_cast<uint8_t>(declare_parameter<int>("system_id", DEFAULT_SYSTEM_ID));
    altitude_m_ = declare_parameter<double>("altitude_m", DEFAULT_ALTITUDE_M);
    forward_m_ = declare_parameter<double>("forward_m", DEFAULT_FORWARD_M);

    if (altitude_m_ <= 0.0) {
      // Refusing to construct means the node never publishes a setpoint, which
      // is the right response to a number that cannot mean anything.
      throw std::invalid_argument("altitude_m must be greater than zero.");
    }

    // -- Topics -----------------------------------------------------------
    // Everything to PX4 goes out on /fmu/in/, everything from PX4 arrives on
    // /fmu/out/. Those names are created by PX4's uXRCE-DDS client; they are
    // not a ROS convention and you cannot rename them from this side.
    const std::string prefix = px4_topic_prefix(px4_namespace);

    // OUT: "here is which part of the setpoint to pay attention to"
    offboard_mode_publisher_ = create_publisher<OffboardControlMode>(
      prefix + "/fmu/in/offboard_control_mode", px4_stream_qos());
    // OUT: "here is where I would like to be"
    setpoint_publisher_ = create_publisher<TrajectorySetpoint>(
      prefix + "/fmu/in/trajectory_setpoint", px4_stream_qos());
    // OUT: one-shot orders -- arm, change mode, land
    command_publisher_ = create_publisher<VehicleCommand>(
      prefix + "/fmu/in/vehicle_command", px4_command_qos());

    // IN: where PX4's estimator thinks the vehicle is
    local_position_subscription_ = create_subscription<VehicleLocalPosition>(
      prefix + "/fmu/out/vehicle_local_position", px4_stream_qos(),
      [this](const VehicleLocalPosition & message) {local_position_ = message;});
    // IN: armed or not, which flight mode, which failsafes are unhappy
    status_subscription_ = create_subscription<VehicleStatus>(
      prefix + "/fmu/out/vehicle_status_v1", px4_stream_qos(),
      [this](const VehicleStatus & message) {vehicle_status_ = message;});

    // -- The clock --------------------------------------------------------
    // rclcpp::create_timer with the node's clock, rather than a wall timer, so
    // the loop follows ROS time. That matters the moment simulated time and
    // wall-clock time stop agreeing. See docs/SIMULATION-FIDELITY.md.
    arming_started_ = now();
    control_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / CONTROL_LOOP_HZ),
      [this]() {run_control_tick();});

    RCLCPP_INFO(
      get_logger(), "will climb to %.1f m, fly %.1f m north, and land. Commanding %s/fmu/.",
      altitude_m_, forward_m_, prefix.c_str());
  }

private:
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

  // -- The control loop ---------------------------------------------------

  /// Runs CONTROL_LOOP_HZ times a second, for the whole flight.
  ///
  /// The order matters. Talking to PX4 comes first and happens unconditionally,
  /// because RULE 2 says that stream must never pause. Only then do we think
  /// about whether it is time to move on to the next phase.
  void run_control_tick()
  {
    if (phase_ != Phase::Landing && phase_ != Phase::Done) {
      // Once we have handed the vehicle to PX4's landing routine, our setpoints
      // are no longer wanted, so the stream stops here and not before.
      publish_offboard_control_mode();
      publish_setpoint(target_position());
    }
    advance_phase();
  }

  /// Where the drone should be right now.
  ///
  /// The whole flight path lives in this one function. Everything except the
  /// forward leg is "hold station directly above the takeoff point", which is
  /// also what we publish while still on the ground, waiting to arm.
  Position target_position() const
  {
    if (phase_ == Phase::FlyingForward) {
      return Position{forward_m_, 0.0, -altitude_m_};
    }
    return Position{0.0, 0.0, -altitude_m_};
  }

  /// The state machine: decides when one phase is finished and the next begins.
  ///
  /// Everything below the guard may assume telemetry has arrived. The accessors
  /// use std::optional::value(), which throws if that is ever untrue, rather
  /// than reading through an empty optional and misbehaving quietly.
  void advance_phase()
  {
    if (!local_position_ || !vehicle_status_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), PROBLEM_REPORT_INTERVAL_MS,
        "No telemetry from the flight controller yet. Is MicroXRCEAgent running, "
        "and did it start before PX4?");
      return;
    }
    warn_if_px4_took_control();

    switch (phase_) {
      case Phase::Priming:
        // RULE 2: do nothing but stream setpoints for a second, so that PX4 has
        // an established stream to accept when we ask for offboard mode.
        if (++priming_ticks_ >= SETPOINTS_BEFORE_ARMING) {
          arming_started_ = now();
          phase_ = Phase::Arming;
        }
        break;

      case Phase::Arming:
        run_arming();
        break;

      case Phase::EngagingOffboard:
        // Ask until it takes. PX4 accepts this within a tick or two once it is
        // armed and seeing setpoints.
        if (is_offboard()) {
          RCLCPP_INFO(get_logger(), "offboard engaged, climbing to %.1f m.", altitude_m_);
          phase_ = Phase::Climbing;
        } else {
          send_vehicle_command(
            VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
            BASE_MODE_CUSTOM_ENABLED, CUSTOM_MAIN_MODE_OFFBOARD);
        }
        break;

      case Phase::Climbing:
        if (has_arrived()) {
          RCLCPP_INFO(get_logger(), "at altitude, flying %.1f m forward.", forward_m_);
          phase_ = Phase::FlyingForward;
        }
        break;

      case Phase::FlyingForward:
        if (has_arrived()) {
          RCLCPP_INFO(get_logger(), "arrived, landing.");
          // Hand the vehicle to PX4's own landing routine. We could fly it down
          // ourselves with setpoints, but PX4's lander already handles ground
          // detection and disarming, and it is the same code the real aircraft
          // runs.
          send_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
          phase_ = Phase::Landing;
        }
        break;

      case Phase::Landing:
        // PX4 disarms itself once it has settled on the ground. That is the only
        // honest signal that the flight is actually over.
        if (!is_armed()) {
          RCLCPP_INFO(get_logger(), "landed and disarmed. Flight complete.");
          phase_ = Phase::Done;
          control_timer_->cancel();
        }
        break;

      case Phase::Done:
        break;
    }
  }

  /// RULE 3: get the vehicle armed, and explain it if PX4 says no.
  ///
  /// A refusal here is almost always a preflight check doing its job. The one
  /// that catches everybody is the datalink check: the x500 airframe sets
  /// NAV_DLL_ACT 2, "return home on datalink loss", and PX4 will not take off
  /// with a failsafe whose precondition is already broken. Supplying the link
  /// with drone-gcs is the fix; switching the failsafe off is not.
  void run_arming()
  {
    if (is_armed()) {
      RCLCPP_INFO(get_logger(), "armed.");
      phase_ = Phase::EngagingOffboard;
      return;
    }

    const double waiting_s = (now() - arming_started_).seconds();
    if (waiting_s > ARMING_TIMEOUT_S) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), PROBLEM_REPORT_INTERVAL_MS,
        "Gave up arming after %.0f s. %s Check `pxh> commander status`.",
        ARMING_TIMEOUT_S, describe_arming_blocker().c_str());
      return;
    }

    if (++arming_attempts_ % ARMING_ATTEMPT_INTERVAL_TICKS == 0) {
      send_vehicle_command(
        VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
        static_cast<float>(VehicleCommand::ARMING_ACTION_ARM));
    }
    if (waiting_s >= ARMING_QUIET_PERIOD_S) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), PROBLEM_REPORT_INTERVAL_MS,
        "not armed yet (%.0f s of %.0f). %s",
        waiting_s, ARMING_TIMEOUT_S, describe_arming_blocker().c_str());
    }
  }

  /// Turns PX4's own view of why it will not arm into something you can act on.
  ///
  /// Worth reading as a list of what PX4 is actually checking before it lets a
  /// vehicle leave the ground.
  std::string describe_arming_blocker() const
  {
    const VehicleStatus & status = vehicle_status_.value();
    if (status.gcs_connection_lost) {
      return "PX4 reports no ground control station link, so its datalink-loss "
             "failsafe is blocking arming. Is drone-gcs running?";
    }
    if (status.failsafe) {
      return "PX4 reports a failsafe active.";
    }
    if (!status.pre_flight_checks_pass) {
      return "PX4 reports preflight checks still failing.";
    }
    return "PX4 reports itself ready, so the command may not be reaching it; "
           "check px4_ns and system_id.";
  }

  /// A failsafe pulls the vehicle out of offboard mode, and PX4 is right to win
  /// that argument. Say so plainly rather than streaming setpoints at a vehicle
  /// that has stopped listening.
  ///
  /// Killing drone-gcs mid-flight is how you watch this happen on purpose.
  void warn_if_px4_took_control() const
  {
    const bool airborne = phase_ == Phase::Climbing || phase_ == Phase::FlyingForward;
    if (airborne && !is_offboard()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), PROBLEM_REPORT_INTERVAL_MS,
        "PX4 left offboard mode (nav_state %u, failsafe %s). It is flying its own "
        "failsafe now and this node is not in control.",
        vehicle_status_.value().nav_state,
        vehicle_status_.value().failsafe ? "active" : "clear");
    }
  }

  // -- Reading the vehicle ------------------------------------------------

  bool is_armed() const
  {
    return vehicle_status_.value().arming_state == VehicleStatus::ARMING_STATE_ARMED;
  }

  bool is_offboard() const
  {
    return vehicle_status_.value().nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  }

  /// Straight-line distance from where PX4 says we are to where we asked to be.
  bool has_arrived() const
  {
    const VehicleLocalPosition & here = local_position_.value();
    const Position target = target_position();
    const double north_error = here.x - target.north_m;
    const double east_error = here.y - target.east_m;
    const double down_error = here.z - target.down_m;
    return std::sqrt(
      north_error * north_error + east_error * east_error +
      down_error * down_error) < ARRIVAL_RADIUS_M;
  }

  // -- Talking to PX4 -----------------------------------------------------

  uint64_t px4_timestamp_us() const
  {
    return static_cast<uint64_t>(get_clock()->now().nanoseconds() / NANOSECONDS_PER_MICROSECOND);
  }

  /// RULE 2, the stream itself.
  ///
  /// This message does not carry a position. It tells PX4 which FIELDS of the
  /// trajectory setpoint to act on, and its arrival is what PX4 uses to decide
  /// that an offboard controller is alive. It must go out on every single tick,
  /// before the mode switch and after it, or offboard is refused or dropped.
  /// The fields we leave alone default to false, so PX4 acts on position only.
  void publish_offboard_control_mode()
  {
    OffboardControlMode message;
    message.position = true;
    message.timestamp = px4_timestamp_us();
    offboard_mode_publisher_->publish(message);
  }

  /// The actual "please be here" message, in metres, in the local NED frame.
  ///
  /// Fields we do not set stay at zero, which PX4 reads as a real command, not
  /// as "unset". That is fine here: we want zero velocity at the target, since
  /// the drone is meant to arrive and stop. A flight that is genuinely moving
  /// through a point wants a real velocity here instead, or NaN, which is how
  /// you tell PX4 not to control something at all.
  void publish_setpoint(const Position & target)
  {
    TrajectorySetpoint message;
    message.position = {
      static_cast<float>(target.north_m),
      static_cast<float>(target.east_m),
      static_cast<float>(target.down_m)};
    message.timestamp = px4_timestamp_us();
    setpoint_publisher_->publish(message);
  }

  /// One MAVLink-style order to this vehicle's flight controller.
  ///
  /// The same message type carries arm, disarm, mode changes, takeoff, land and
  /// a hundred other commands; `command` picks which, and param1..param7 mean
  /// different things for each one.
  ///   https://mavlink.io/en/messages/common.html
  void send_vehicle_command(uint32_t command, float param1 = 0.0F, float param2 = 0.0F)
  {
    VehicleCommand message;
    message.command = command;
    message.param1 = param1;
    message.param2 = param2;
    // Addressing. PX4 drops commands aimed at a different system id, which is
    // what stops one drone in a swarm from arming all the others.
    message.target_system = system_id_;
    message.target_component = PX4_COMPONENT_ID;
    message.source_system = system_id_;
    message.source_component = PX4_COMPONENT_ID;
    message.from_external = true;   // "this came from a companion computer"
    message.timestamp = px4_timestamp_us();
    command_publisher_->publish(message);
  }

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

}  // namespace drone_swarm

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    // make_shared, and rclcpp owns it from here. There is no new or delete in
    // this file and nothing to free by hand.
    rclcpp::spin(std::make_shared<drone_swarm::Hop>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("hop"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
