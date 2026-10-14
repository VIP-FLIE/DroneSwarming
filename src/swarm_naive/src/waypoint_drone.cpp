// ===========================================================================
// waypoint_drone.cpp -- the WaypointDrone node's method bodies.
//
// The declarations, the tunables and the three rules PX4 imposes are in
// include/swarm_naive/waypoint_drone.hpp. main() is in main.cpp.
// ===========================================================================

#include "swarm_naive/waypoint_drone.hpp"

#include <cmath>
#include <stdexcept>

namespace swarm_naive
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

WaypointDrone::WaypointDrone()
: Node("waypoint_drone")
{
  // -- Which aircraft am I talking to? ----------------------------------
  // These three come from the launch file and are the ones that will bite
  // you if they disagree with how PX4 was started.
  const std::string px4_namespace = declare_parameter<std::string>("px4_ns", "");
  system_id_ = static_cast<uint8_t>(declare_parameter<int>("system_id", 1));

  // -- Which drone in the swarm am I? -----------------------------------
  slot_ = declare_parameter<int>("slot", 0);
  count_ = declare_parameter<int>("count", 3);

  // -- Where is my pad, in the shared swarm frame? ----------------------
  // The launch file spawns this drone here AND passes the same numbers here.
  // Those two have to agree or the whole formation is wrong, which is why a
  // single launch file owns both. On a real aircraft this number comes from a
  // survey or an RTK fix instead.
  home_.north_m = declare_parameter<double>("home_north_m", 0.0);
  home_.east_m = declare_parameter<double>("home_east_m", 0.0);
  home_.down_m = 0.0;

  // -- The shared flight plan -------------------------------------------
  ring_centre_north_m_ = declare_parameter<double>("centre_north_m", 10.0);
  ring_centre_east_m_ = declare_parameter<double>("centre_east_m", 0.0);
  radius_m_ = declare_parameter<double>("radius_m", 8.0);
  altitude_m_ = declare_parameter<double>("altitude_m", 4.0);
  layer_m_ = declare_parameter<double>("layer_m", 2.0);
  laps_ = declare_parameter<int>("laps", 1);

  validate_parameters();

  // Built once, in the constructor, because it never changes. A drone that
  // recomputes its plan every tick is a drone whose plan you cannot print.
  route_ = build_route();

  // -- Topics -----------------------------------------------------------
  // Everything to PX4 goes out on /fmu/in/, everything from PX4 arrives on
  // /fmu/out/. Those names come from PX4's uXRCE-DDS client and cannot be
  // renamed from this side. Note that every one of them is prefixed: this is
  // the only thing that stops drone 2's setpoints from flying drone 1.
  const std::string prefix = px4_topic_prefix(px4_namespace);

  offboard_mode_publisher_ = create_publisher<OffboardControlMode>(
    prefix + "/fmu/in/offboard_control_mode", px4_stream_qos());
  setpoint_publisher_ = create_publisher<TrajectorySetpoint>(
    prefix + "/fmu/in/trajectory_setpoint", px4_stream_qos());
  command_publisher_ = create_publisher<VehicleCommand>(
    prefix + "/fmu/in/vehicle_command", px4_command_qos());

  local_position_subscription_ = create_subscription<VehicleLocalPosition>(
    prefix + "/fmu/out/vehicle_local_position", px4_stream_qos(),
    [this](const VehicleLocalPosition & message) {local_position_ = message;});
  status_subscription_ = create_subscription<VehicleStatus>(
    prefix + "/fmu/out/vehicle_status_v1", px4_stream_qos(),
    [this](const VehicleStatus & message) {vehicle_status_ = message;});

  // -- The clock --------------------------------------------------------
  // rclcpp::create_timer with the node's clock rather than a wall timer, so
  // the loop follows ROS time. That matters the moment simulated time and
  // wall-clock time stop agreeing. See docs/simulation/SIMULATION-FIDELITY.md.
  arming_started_ = now();
  leg_started_ = now();
  control_timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(1.0 / CONTROL_LOOP_HZ),
    [this]() {run_control_tick();});

  announce_plan(prefix);
}

void WaypointDrone::validate_parameters() const
{
  // Throwing here means the node never publishes a setpoint, which is the
  // right response: a drone that takes off with a nonsense plan is worse than
  // a drone that never leaves the ground.
  if (count_ < 1 || count_ > MAX_SWARM_SIZE) {
    throw std::invalid_argument(
            "count must be between 1 and " + std::to_string(MAX_SWARM_SIZE) +
            ". PX4 instance N uses MAVLink system ID N + 1.");
  }
  if (slot_ < 0 || slot_ >= count_) {
    throw std::invalid_argument("slot must be in the range 0 to count-1.");
  }
  if (altitude_m_ <= 0.0) {
    throw std::invalid_argument("altitude_m must be greater than zero.");
  }
  if (layer_m_ < 0.0) {
    throw std::invalid_argument("layer_m cannot be negative.");
  }
  if (radius_m_ <= 0.0) {
    throw std::invalid_argument("radius_m must be greater than zero.");
  }
  if (laps_ < 1) {
    throw std::invalid_argument("laps must be at least 1.");
  }
}

double WaypointDrone::station_altitude_m() const
{
  // This is a guess baked in before takeoff. It works because the plan is
  // known in advance and nothing ever deviates from it. Both halves of that
  // sentence stop being true on real hardware.
  return altitude_m_ + static_cast<double>(slot_) * layer_m_;
}

SwarmPoint WaypointDrone::ring_vertex(int index, double down_m) const
{
  // The ring has one vertex per drone, evenly spaced, measured clockwise from
  // north so vertex 0 is due north of the centre. Indices past count_ wrap
  // round on their own because cosine and sine are periodic, which is why
  // build_route() can just keep counting upwards.
  const double angle_rad = 2.0 * M_PI * static_cast<double>(index) /
    static_cast<double>(count_);
  return SwarmPoint{
    ring_centre_north_m_ + radius_m_ * std::cos(angle_rad),
    ring_centre_east_m_ + radius_m_ * std::sin(angle_rad),
    down_m};
}

std::vector<SwarmPoint> WaypointDrone::build_route() const
{
  // Written in the shared frame on purpose. Nothing in here mentions where
  // this particular drone's pad is except the first and last entries, so the
  // ring is genuinely the same object for every drone in the swarm.
  const double cruise_down_m = -station_altitude_m();
  std::vector<SwarmPoint> route;

  // 1. Straight up off the pad. Climbing vertically before going anywhere
  //    keeps drones from cutting across each other's pads on the way out.
  route.push_back(SwarmPoint{home_.north_m, home_.east_m, cruise_down_m});

  // 2. Once round the ring, entering at this drone's own vertex. Visiting
  //    count_ * laps_ + 1 vertices means the drone finishes back where it
  //    joined, so every drone flies exactly the same distance.
  const int vertices_to_visit = count_ * laps_;
  for (int step = 0; step <= vertices_to_visit; ++step) {
    route.push_back(ring_vertex(slot_ + step, cruise_down_m));
  }

  // 3. Back over its own pad, and land from there.
  route.push_back(SwarmPoint{home_.north_m, home_.east_m, cruise_down_m});
  return route;
}

void WaypointDrone::announce_plan(const std::string & prefix) const
{
  RCLCPP_INFO(
    get_logger(),
    "slot %d of %d. Pad at north %.1f east %.1f, flying at %.1f m, "
    "%zu waypoints round a %.1f m ring. Commanding %s/fmu/ as system %u.",
    slot_, count_, home_.north_m, home_.east_m, station_altitude_m(),
    route_.size(), radius_m_, prefix.empty() ? "" : prefix.c_str(),
    static_cast<unsigned>(system_id_));
}

void WaypointDrone::run_control_tick()
{
  // The order matters. Talking to PX4 comes first and happens unconditionally,
  // because RULE 2 says that stream must never pause. Only then do we think
  // about whether it is time to move on.
  if (phase_ != Phase::Landing && phase_ != Phase::Done) {
    // Once PX4's landing routine has the vehicle our setpoints are no longer
    // wanted, so the stream stops there and not before.
    publish_offboard_control_mode();
    publish_setpoint(to_local(current_target()));
  }
  advance_phase();
}

SwarmPoint WaypointDrone::current_target() const
{
  // Before the flight starts that is the first waypoint, which is the point
  // directly above its own pad. Publishing a setpoint it is not at yet is
  // harmless: PX4 is not acting on it until offboard is engaged, and RULE 2
  // needs the stream to exist regardless.
  return route_.at(phase_ == Phase::Flying ? route_index_ : 0U);
}

LocalPoint WaypointDrone::to_local(const SwarmPoint & point) const
{
  // The route is written in the shared swarm frame, whose origin is drone 0's
  // pad. PX4 only understands this vehicle's own local frame, whose origin is
  // this vehicle's own pad. Subtracting the pad offset moves a point from one
  // to the other.
  //
  // Verified in the simulator: three vehicles spawned 6 m apart each report
  // vehicle_local_position x and y within 3 cm of zero, so each really is
  // measuring from its own pad and not from a shared world origin.
  //
  // down_m is passed through untouched, because every pad here is on the same
  // flat ground and so shares an altitude datum. On sloping ground each drone
  // would need a third offset for exactly the same reason it needs these two.
  return LocalPoint{
    point.north_m - home_.north_m,
    point.east_m - home_.east_m,
    point.down_m};
}

void WaypointDrone::advance_phase()
{
  // Everything below the guard may assume telemetry has arrived. The accessors
  // use std::optional::value(), which throws if that is ever untrue, rather
  // than reading through an empty optional and misbehaving quietly.
  if (!local_position_ || !vehicle_status_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), PROBLEM_REPORT_INTERVAL_MS,
      "No telemetry from the flight controller yet. Is MicroXRCEAgent running, "
      "did it start before PX4, and is px4_ns right for this instance?");
    return;
  }
  if (!yaw_setpoint_rad_) {
    // Start out pointing wherever the aircraft is already pointing, so it does
    // not spin on the pad before it has anywhere to go.
    yaw_setpoint_rad_ = static_cast<double>(local_position_->heading);
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
        RCLCPP_INFO(
          get_logger(), "offboard engaged, climbing to %.1f m.",
          station_altitude_m());
        leg_started_ = now();
        phase_ = Phase::Flying;
      } else {
        send_vehicle_command(
          VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
          BASE_MODE_CUSTOM_ENABLED, CUSTOM_MAIN_MODE_OFFBOARD);
      }
      break;

    case Phase::Flying:
      run_flying();
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

void WaypointDrone::run_arming()
{
  // A refusal here is almost always a preflight check doing its job. The one
  // that catches everybody in a swarm is the datalink check: the x500 airframe
  // sets NAV_DLL_ACT 2, "return home on datalink loss", and PX4 will not take
  // off with a failsafe whose precondition is already broken. Each vehicle
  // enforces it separately. QGroundControl supplies the ground station link;
  // the failsafe remains enabled.
  if (is_armed()) {
    RCLCPP_INFO(get_logger(), "armed.");
    phase_ = Phase::EngagingOffboard;
    return;
  }

  const double waiting_s = (now() - arming_started_).seconds();
  if (waiting_s > ARMING_TIMEOUT_S) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), PROBLEM_REPORT_INTERVAL_MS,
      "Gave up arming after %.0f s. %s Check `commander status` on this "
      "instance's console.", ARMING_TIMEOUT_S, describe_arming_blocker().c_str());
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

void WaypointDrone::run_flying()
{
  // This is the entire flight logic of the package. Everything that makes it a
  // swarm rather than a single drone happened before takeoff, in build_route()
  // and to_local().
  if (!has_arrived()) {
    warn_if_leg_is_stalled();
    return;
  }

  ++route_index_;
  leg_started_ = now();

  if (route_index_ >= route_.size()) {
    RCLCPP_INFO(get_logger(), "route complete over its own pad, landing.");
    // Hand the vehicle to PX4's own landing routine rather than flying it
    // down with setpoints. PX4's lander already handles ground detection and
    // disarming, and it is the same code the real aircraft runs.
    send_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
    phase_ = Phase::Landing;
    return;
  }

  const SwarmPoint & target = route_[route_index_];
  RCLCPP_INFO(
    get_logger(), "waypoint %zu of %zu: swarm frame north %.1f east %.1f alt %.1f",
    route_index_, route_.size() - 1U, target.north_m, target.east_m, -target.down_m);
}

void WaypointDrone::warn_if_leg_is_stalled() const
{
  if ((now() - leg_started_).seconds() < LEG_STALL_WARNING_S) {
    return;
  }
  const LocalPoint target = to_local(current_target());
  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), PROBLEM_REPORT_INTERVAL_MS,
    "still %.1f m from waypoint %zu after %.0f s. nav_state %u, %s.",
    distance_to(target), route_index_, (now() - leg_started_).seconds(),
    vehicle_status_.value().nav_state,
    is_offboard() ? "still offboard" : "NOT in offboard mode any more");
}

void WaypointDrone::warn_if_px4_took_control() const
{
  // Closing QGroundControl mid-flight makes PX4 apply its configured failsafe.
  if (phase_ == Phase::Flying && !is_offboard()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), PROBLEM_REPORT_INTERVAL_MS,
      "PX4 left offboard mode (nav_state %u, failsafe %s). It is flying its own "
      "failsafe now and this node is not in control.",
      vehicle_status_.value().nav_state,
      vehicle_status_.value().failsafe ? "active" : "clear");
  }
}

std::string WaypointDrone::describe_arming_blocker() const
{
  const VehicleStatus & status = vehicle_status_.value();
  if (status.gcs_connection_lost) {
    return "PX4 reports no ground control station link, so its datalink-loss "
           "failsafe is blocking arming. Check QGroundControl's UDP link.";
  }
  if (status.failsafe) {
    return "PX4 reports a failsafe active.";
  }
  if (!status.pre_flight_checks_pass) {
    return "PX4 reports preflight checks still failing.";
  }
  return "PX4 reports itself ready, so the command may not be reaching it; "
         "check that system_id is this instance's number plus one.";
}

bool WaypointDrone::is_armed() const
{
  return vehicle_status_.value().arming_state == VehicleStatus::ARMING_STATE_ARMED;
}

bool WaypointDrone::is_offboard() const
{
  return vehicle_status_.value().nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD;
}

double WaypointDrone::distance_to(const LocalPoint & target) const
{
  const VehicleLocalPosition & here = local_position_.value();
  const double north_error = static_cast<double>(here.x) - target.north_m;
  const double east_error = static_cast<double>(here.y) - target.east_m;
  const double down_error = static_cast<double>(here.z) - target.down_m;
  return std::sqrt(
    north_error * north_error + east_error * east_error + down_error * down_error);
}

bool WaypointDrone::has_arrived() const
{
  return distance_to(to_local(current_target())) < ARRIVAL_RADIUS_M;
}

uint64_t WaypointDrone::px4_timestamp_us() const
{
  return static_cast<uint64_t>(get_clock()->now().nanoseconds() / NANOSECONDS_PER_MICROSECOND);
}

void WaypointDrone::publish_offboard_control_mode()
{
  // This message carries no position. It tells PX4 which FIELDS of the
  // trajectory setpoint to act on, and its arrival is what PX4 uses to decide
  // that an offboard controller is alive. It must go out on every tick, before
  // the mode switch and after it, or offboard is refused or dropped. Fields we
  // leave alone default to false, so PX4 acts on position only.
  OffboardControlMode message;
  message.position = true;
  message.timestamp = px4_timestamp_us();
  offboard_mode_publisher_->publish(message);
}

void WaypointDrone::publish_setpoint(const LocalPoint & target)
{
  // Fields we do not set stay at zero, and PX4 reads zero as a real command
  // rather than as "unset". That is what we want for velocity here, since each
  // drone is meant to arrive at a waypoint and stop. To tell PX4 not to control
  // something at all, set it to NaN.
  TrajectorySetpoint message;
  message.position = {
    static_cast<float>(target.north_m),
    static_cast<float>(target.east_m),
    static_cast<float>(target.down_m)};
  message.yaw = static_cast<float>(update_yaw_setpoint(target));
  message.timestamp = px4_timestamp_us();
  setpoint_publisher_->publish(message);
}

double WaypointDrone::update_yaw_setpoint(const LocalPoint & target)
{
  // Not required for the flight: PX4 will happily fly a quadrotor sideways,
  // and hop.cpp leaves yaw at its default of zero, which commands "face
  // north". Steering it is worth the six lines because a swarm that faces
  // where it is going is far easier to read on screen, and because a real
  // aircraft with a forward-facing camera or a fixed antenna has to do it.
  //
  // The deadband matters: the bearing to a point you are already sitting on is
  // meaningless, so inside YAW_DEADBAND_M we hold the last commanded heading
  // instead of chasing noise.
  if (!local_position_) {
    return yaw_setpoint_rad_.value_or(0.0);
  }
  const VehicleLocalPosition & here = local_position_.value();
  const double north_error = target.north_m - static_cast<double>(here.x);
  const double east_error = target.east_m - static_cast<double>(here.y);
  if (std::hypot(north_error, east_error) >= YAW_DEADBAND_M) {
    // atan2(east, north) because yaw in NED is measured from north towards
    // east, which is the opposite argument order to the usual maths one.
    yaw_setpoint_rad_ = std::atan2(east_error, north_error);
  }
  return yaw_setpoint_rad_.value_or(0.0);
}

void WaypointDrone::send_vehicle_command(uint32_t command, float param1, float param2)
{
  // The same message type carries arm, disarm, mode changes, takeoff, land and
  // a hundred other commands; `command` picks which, and param1..param7 mean
  // different things for each one.
  //   https://mavlink.io/en/messages/common.html
  VehicleCommand message;
  message.command = command;
  message.param1 = param1;
  message.param2 = param2;
  // Addressing, and the reason a swarm needs it. PX4 drops any command aimed
  // at a different system id, which is the only thing stopping this node from
  // arming every other drone on the field. SITL instance N answers to system
  // id N + 1, so a wrong system_id looks like a node that runs
  // perfectly while nothing ever arms.
  message.target_system = system_id_;
  message.target_component = PX4_COMPONENT_ID;
  message.source_system = system_id_;
  message.source_component = PX4_COMPONENT_ID;
  message.from_external = true;   // "this came from a companion computer"
  message.timestamp = px4_timestamp_us();
  command_publisher_->publish(message);
}

}  // namespace swarm_naive
