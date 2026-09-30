// ===========================================================================
// hop.cpp -- the Hop node's method bodies.
//
// The declarations, the tunables and the three rules PX4 imposes are in
// include/cpp_single_hop/hop.hpp. main() is in main.cpp.
// ===========================================================================

#include "cpp_single_hop/hop.hpp"

#include <cmath>
#include <stdexcept>

namespace cpp_single_hop
{

// ---------------------------------------------------------------------------
// RULE 1: QoS. See the header for why these two profiles differ.
// ---------------------------------------------------------------------------

rclcpp::QoS px4_stream_qos()
{
  rclcpp::QoS qos{rclcpp::KeepLast(1)};
  qos.best_effort();
  qos.transient_local();
  return qos;
}

rclcpp::QoS px4_command_qos()
{
  rclcpp::QoS qos{rclcpp::KeepLast(1)};
  qos.reliable();
  qos.transient_local();
  return qos;
}

std::string px4_topic_prefix(const std::string & px4_namespace)
{
  const auto first = px4_namespace.find_first_not_of('/');
  if (first == std::string::npos) {
    return "";
  }
  const auto last = px4_namespace.find_last_not_of('/');
  return "/" + px4_namespace.substr(first, last - first + 1);
}

// ===========================================================================
// The node.
// ===========================================================================

Hop::Hop() : Node("hop")
{
  // -- Parameters -------------------------------------------------------
  // Declared with defaults, overridable from the command line:
  //   ros2 run cpp_single_hop hop --ros-args -p altitude_m:=5.0
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

void Hop::run_control_tick()
{
  // The order matters. Talking to PX4 comes first and happens unconditionally,
  // because RULE 2 says that stream must never pause. Only then do we think
  // about whether it is time to move on to the next phase.
  if (phase_ != Phase::Landing && phase_ != Phase::Done) {
    // Once we have handed the vehicle to PX4's landing routine, our setpoints
    // are no longer wanted, so the stream stops here and not before.
    publish_offboard_control_mode();
    publish_setpoint(target_position());
  }
  advance_phase();
}

Position Hop::target_position() const
{
  // The whole flight path lives in this one function. Everything except the
  // forward leg is "hold station directly above the takeoff point", which is
  // also what we publish while still on the ground, waiting to arm.
  if (phase_ == Phase::FlyingForward) {
    return Position{forward_m_, 0.0, -altitude_m_};
  }
  return Position{0.0, 0.0, -altitude_m_};
}

void Hop::advance_phase()
{
  // Everything below the guard may assume telemetry has arrived. The accessors
  // use std::optional::value(), which throws if that is ever untrue, rather
  // than reading through an empty optional and misbehaving quietly.
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

void Hop::run_arming()
{
  // A refusal here is almost always a preflight check doing its job. The one
  // that catches everybody is the datalink check: the x500 airframe sets
  // NAV_DLL_ACT 2, "return home on datalink loss", and PX4 will not take off
  // with a failsafe whose precondition is already broken. Supplying the link
  // with drone-gcs is the fix; switching the failsafe off is not.
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

std::string Hop::describe_arming_blocker() const
{
  // Worth reading as a list of what PX4 is actually checking before it lets a
  // vehicle leave the ground.
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

void Hop::warn_if_px4_took_control() const
{
  // A failsafe pulls the vehicle out of offboard mode, and PX4 is right to win
  // that argument. Say so plainly rather than streaming setpoints at a vehicle
  // that has stopped listening.
  //
  // Killing drone-gcs mid-flight is how you watch this happen on purpose.
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

bool Hop::is_armed() const
{
  return vehicle_status_.value().arming_state == VehicleStatus::ARMING_STATE_ARMED;
}

bool Hop::is_offboard() const
{
  return vehicle_status_.value().nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD;
}

bool Hop::has_arrived() const
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

uint64_t Hop::px4_timestamp_us() const
{
  return static_cast<uint64_t>(get_clock()->now().nanoseconds() / NANOSECONDS_PER_MICROSECOND);
}

void Hop::publish_offboard_control_mode()
{
  // RULE 2, the stream itself.
  //
  // This message does not carry a position. It tells PX4 which FIELDS of the
  // trajectory setpoint to act on, and its arrival is what PX4 uses to decide
  // that an offboard controller is alive. It must go out on every single tick,
  // before the mode switch and after it, or offboard is refused or dropped.
  // The fields we leave alone default to false, so PX4 acts on position only.
  OffboardControlMode message;
  message.position = true;
  message.timestamp = px4_timestamp_us();
  offboard_mode_publisher_->publish(message);
}

void Hop::publish_setpoint(const Position & target)
{
  // Fields we do not set stay at zero, which PX4 reads as a real command, not
  // as "unset". That is fine here: we want zero velocity at the target, since
  // the drone is meant to arrive and stop. A flight that is genuinely moving
  // through a point wants a real velocity here instead, or NaN, which is how
  // you tell PX4 not to control something at all.
  TrajectorySetpoint message;
  message.position = {
    static_cast<float>(target.north_m),
    static_cast<float>(target.east_m),
    static_cast<float>(target.down_m)};
  message.timestamp = px4_timestamp_us();
  setpoint_publisher_->publish(message);
}

void Hop::send_vehicle_command(uint32_t command, float param1, float param2)
{
  // The same message type carries arm, disarm, mode changes, takeoff, land and
  // a hundred other commands; `command` picks which, and param1..param7 mean
  // different things for each one.
  //   https://mavlink.io/en/messages/common.html
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

}  // namespace cpp_single_hop
