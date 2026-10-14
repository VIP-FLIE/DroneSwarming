// ===========================================================================
// formation_drone.cpp -- the FormationDrone node's method bodies.
//
// The declarations, the tunables and the PX4 handshake are in
// include/swarm_comms/formation_drone.hpp. main() is in main.cpp.
// ===========================================================================

#include "swarm_comms/formation_drone.hpp"

#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace swarm_comms
{

// ---------------------------------------------------------------------------
// RULE 1: QoS. See the header for why these profiles differ.
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

rclcpp::QoS swarm_qos(int count)
{
  rclcpp::QoS qos{rclcpp::KeepLast(static_cast<size_t>(2 * count))};
  qos.best_effort();
  qos.durability_volatile();
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

FormationDrone::FormationDrone()
: Node("formation_drone")
{
  // -- Which aircraft am I talking to? ----------------------------------
  const std::string px4_namespace = declare_parameter<std::string>("px4_ns", "");
  system_id_ = static_cast<uint8_t>(declare_parameter<int>("system_id", 1));

  // -- Which drone in the swarm am I? -----------------------------------
  slot_ = declare_parameter<int>("slot", 0);
  count_ = declare_parameter<int>("count", 3);

  // -- Where is my pad, in the shared swarm frame? ----------------------
  // Still a launch-time parameter, exactly as in swarm_naive. Communication
  // does not remove the need to know where you started; it only lets drones
  // tell each other where they are NOW. On real aircraft this number comes
  // from a survey or an RTK fix.
  home_.north_m = declare_parameter<double>("home_north_m", 0.0);
  home_.east_m = declare_parameter<double>("home_east_m", 0.0);
  home_.down_m = 0.0;

  // -- The shared flight plan, which only the leader actually flies -----
  ring_centre_north_m_ = declare_parameter<double>("centre_north_m", 10.0);
  ring_centre_east_m_ = declare_parameter<double>("centre_east_m", 0.0);
  radius_m_ = declare_parameter<double>("radius_m", 8.0);
  altitude_m_ = declare_parameter<double>("altitude_m", 4.0);
  layer_m_ = declare_parameter<double>("layer_m", 2.0);
  laps_ = declare_parameter<int>("laps", 1);

  // -- The formation ----------------------------------------------------
  follow_distance_m_ = declare_parameter<double>("follow_distance_m", 5.0);

  validate_parameters();

  // Only the leader has a route. Everybody else's target is a function of
  // somebody else's position and cannot be known in advance, which is rather
  // the point.
  if (is_leader()) {
    route_ = build_route();
  }

  // One entry per slot, so a neighbour lookup is an array index rather than a
  // map lookup. Empty means that drone has never been heard from.
  neighbours_.resize(static_cast<size_t>(count_));

  // -- Topics: PX4 --------------------------------------------------------
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

  // -- Topics: the swarm --------------------------------------------------
  // One topic, every drone on it, no namespace. Note there is no separate
  // topic per drone and no point-to-point addressing: a broadcast that
  // everybody can hear is both simpler and closer to how a radio actually
  // behaves.
  swarm_publisher_ = create_publisher<SwarmState>(
    SWARM_STATE_TOPIC, swarm_qos(count_));
  swarm_subscription_ = create_subscription<SwarmState>(
    SWARM_STATE_TOPIC, swarm_qos(count_),
    [this](const SwarmState & message) {on_swarm_state(message);});

  // -- The clock ----------------------------------------------------------
  arming_started_ = now();
  leg_started_ = now();
  control_timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(1.0 / CONTROL_LOOP_HZ),
    [this]() {run_control_tick();});

  announce_plan(prefix);
}

bool FormationDrone::is_leader() const {return slot_ == 0;}

void FormationDrone::validate_parameters() const
{
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
  if (follow_distance_m_ <= MIN_CHASE_BASELINE_M) {
    throw std::invalid_argument(
            "follow_distance_m must be more than " +
            std::to_string(MIN_CHASE_BASELINE_M) +
            " m, or the drone ahead is inside the deadband and the formation "
            "never converges.");
  }
}

double FormationDrone::station_altitude_m() const
{
  return altitude_m_ + static_cast<double>(slot_) * layer_m_;
}

SwarmPoint FormationDrone::ring_vertex(int index, double down_m) const
{
  const double angle_rad = 2.0 * M_PI * static_cast<double>(index) /
    static_cast<double>(count_);
  return SwarmPoint{
    ring_centre_north_m_ + radius_m_ * std::cos(angle_rad),
    ring_centre_east_m_ + radius_m_ * std::sin(angle_rad),
    down_m};
}

std::vector<SwarmPoint> FormationDrone::build_route() const
{
  const double cruise_down_m = -station_altitude_m();
  std::vector<SwarmPoint> route;
  const int vertices_to_visit = count_ * laps_;
  for (int step = 0; step <= vertices_to_visit; ++step) {
    route.push_back(ring_vertex(step, cruise_down_m));
  }
  return route;
}

void FormationDrone::announce_plan(const std::string & prefix) const
{
  if (is_leader()) {
    RCLCPP_INFO(
      get_logger(),
      "slot 0 of %d, LEADER. Pad at north %.1f east %.1f, flying at %.1f m, "
      "%zu waypoints round a %.1f m ring. Commanding %s/fmu/ as system %u.",
      count_, home_.north_m, home_.east_m, station_altitude_m(), route_.size(),
      radius_m_, prefix.empty() ? "" : prefix.c_str(),
      static_cast<unsigned>(system_id_));
  } else {
    RCLCPP_INFO(
      get_logger(),
      "slot %d of %d, following slot %d at %.1f m. Pad at north %.1f east %.1f, "
      "flying at %.1f m. Commanding %s/fmu/ as system %u.",
      slot_, count_, slot_ - 1, follow_distance_m_, home_.north_m, home_.east_m,
      station_altitude_m(), prefix.empty() ? "" : prefix.c_str(),
      static_cast<unsigned>(system_id_));
  }
}

LocalPoint FormationDrone::to_local(const SwarmPoint & point) const
{
  return LocalPoint{
    point.north_m - home_.north_m,
    point.east_m - home_.east_m,
    point.down_m};
}

SwarmPoint FormationDrone::to_swarm(const LocalPoint & point) const
{
  return SwarmPoint{
    point.north_m + home_.north_m,
    point.east_m + home_.east_m,
    point.down_m};
}

SwarmPoint FormationDrone::my_swarm_position() const
{
  const VehicleLocalPosition & here = local_position_.value();
  return to_swarm(
    LocalPoint{
      static_cast<double>(here.x),
      static_cast<double>(here.y),
      static_cast<double>(here.z)});
}

void FormationDrone::on_swarm_state(const SwarmState & message)
{
  // Our own broadcast comes back to us, because we subscribe to the topic we
  // publish on. Dropping it here means the rest of the code can treat
  // neighbours_ as "everybody except me" without checking.
  if (static_cast<int>(message.slot) == slot_) {
    return;
  }
  if (static_cast<int>(message.slot) >= count_) {
    // A slot number we were never told about. Either somebody launched a
    // bigger swarm against the same ROS_DOMAIN_ID, or a stale node from the
    // last run is still up. Both are worth saying out loud, because the
    // symptom otherwise is a formation reacting to an aircraft you cannot see.
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), PROBLEM_REPORT_INTERVAL_MS,
      "Heard slot %u, but this swarm only has %d drones. Another swarm on the "
      "same ROS_DOMAIN_ID, or a node left over from the last run?",
      static_cast<unsigned>(message.slot), count_);
    return;
  }
  std::optional<Neighbour> & entry = neighbours_[static_cast<size_t>(message.slot)];
  if (entry) {
    entry->previous = entry->state;
    entry->state = message;
    entry->received = now();
  } else {
    entry = Neighbour{message, now(), std::nullopt};
  }
}

void FormationDrone::publish_swarm_state()
{
  if (!local_position_) {
    return;
  }
  const SwarmPoint me = my_swarm_position();

  SwarmState message;
  message.stamp_us = px4_timestamp_us();
  message.slot = static_cast<uint8_t>(slot_);
  message.ready = at_station_;
  message.done = done_;
  message.north_m = static_cast<float>(me.north_m);
  message.east_m = static_cast<float>(me.east_m);
  message.down_m = static_cast<float>(me.down_m);
  swarm_publisher_->publish(message);
}

double FormationDrone::link_age_s(int slot) const
{
  const auto & entry = neighbours_.at(static_cast<size_t>(slot));
  if (!entry) {
    return std::numeric_limits<double>::infinity();
  }
  return (now() - entry->received).seconds();
}

std::optional<SwarmState> FormationDrone::fresh_state(int slot) const
{
  if (link_age_s(slot) > LINK_STALE_S) {
    return std::nullopt;
  }
  return neighbours_.at(static_cast<size_t>(slot))->state;
}

void FormationDrone::run_control_tick()
{
  if (phase_ != Phase::Landing && phase_ != Phase::Done) {
    publish_offboard_control_mode();
    publish_setpoint(to_local(current_target()));
  }
  publish_swarm_state();
  advance_phase();
}

SwarmPoint FormationDrone::current_target() const
{
  switch (phase_) {
    case Phase::Flying:
      return is_leader() ? route_.at(route_index_) : follower_target_;

    case Phase::Priming:
    case Phase::Arming:
    case Phase::EngagingOffboard:
    case Phase::Climbing:
    case Phase::WaitingForSwarm:
    case Phase::Returning:
    case Phase::Landing:
    case Phase::Done:
      break;
  }
  // Everything else is "hold station directly above my own pad", which is
  // also what we publish while still on the ground waiting to arm.
  return station_point();
}

SwarmPoint FormationDrone::station_point() const
{
  return SwarmPoint{home_.north_m, home_.east_m, -station_altitude_m()};
}

void FormationDrone::advance_phase()
{
  if (!local_position_ || !vehicle_status_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), PROBLEM_REPORT_INTERVAL_MS,
      "No telemetry from the flight controller yet. Is MicroXRCEAgent running, "
      "did it start before PX4, and is px4_ns right for this instance?");
    return;
  }
  if (!yaw_setpoint_rad_) {
    yaw_setpoint_rad_ = static_cast<double>(local_position_->heading);
  }
  warn_if_px4_took_control();

  switch (phase_) {
    case Phase::Priming:
      if (++priming_ticks_ >= SETPOINTS_BEFORE_ARMING) {
        arming_started_ = now();
        phase_ = Phase::Arming;
      }
      break;

    case Phase::Arming:
      run_arming();
      break;

    case Phase::EngagingOffboard:
      if (is_offboard()) {
        RCLCPP_INFO(
          get_logger(), "offboard engaged, climbing to %.1f m.",
          station_altitude_m());
        leg_started_ = now();
        phase_ = Phase::Climbing;
      } else {
        send_vehicle_command(
          VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
          BASE_MODE_CUSTOM_ENABLED, CUSTOM_MAIN_MODE_OFFBOARD);
      }
      break;

    case Phase::Climbing:
      if (has_arrived()) {
        // Setting this is what makes our broadcast say ready = true, which is
        // the only thing any other drone is waiting for.
        at_station_ = true;
        barrier_started_ = now();
        RCLCPP_INFO(get_logger(), "at station, waiting for the rest of the swarm.");
        phase_ = Phase::WaitingForSwarm;
      } else {
        warn_if_leg_is_stalled();
      }
      break;

    case Phase::WaitingForSwarm:
      run_waiting_for_swarm();
      break;

    case Phase::Flying:
      run_flying();
      check_separation();
      break;

    case Phase::Returning:
      if (has_arrived()) {
        RCLCPP_INFO(get_logger(), "over its own pad, landing.");
        send_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
        phase_ = Phase::Landing;
      } else {
        warn_if_leg_is_stalled();
      }
      break;

    case Phase::Landing:
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

void FormationDrone::run_waiting_for_swarm()
{
  const std::string missing = slots_not_ready();
  if (missing.empty()) {
    // RCLCPP_INFO is printf-style, so a std::string has to be handed over as
    // c_str(). Passing the object straight in compiles with a warning and
    // then prints rubbish, because printf cannot know what a std::string is.
    const std::string plan = is_leader() ?
      std::string("Leading off round the ring.") :
      "Following slot " + std::to_string(slot_ - 1) + ".";
    RCLCPP_INFO(get_logger(), "whole swarm is ready. %s", plan.c_str());
    leg_started_ = now();
    // A follower has nowhere to go until it hears from the drone ahead, so
    // start it holding station rather than at an uninitialised point.
    follower_target_ = station_point();
    phase_ = Phase::Flying;
    return;
  }

  const double waiting_s = (now() - barrier_started_).seconds();
  if (waiting_s > BARRIER_TIMEOUT_S) {
    RCLCPP_ERROR(
      get_logger(),
      "Gave up after %.0f s waiting for %s. Landing rather than hovering on a "
      "swarm that is not going to assemble.", waiting_s, missing.c_str());
    phase_ = Phase::Returning;
    return;
  }
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), PROBLEM_REPORT_INTERVAL_MS,
    "holding at station, waiting for %s (%.0f s of %.0f).",
    missing.c_str(), waiting_s, BARRIER_TIMEOUT_S);
}

std::string FormationDrone::slots_not_ready() const
{
  std::ostringstream missing;
  if (!at_station_) {
    missing << "slot " << slot_ << " (this one)";
  }
  for (int slot = 0; slot < count_; ++slot) {
    if (slot == slot_) {
      continue;
    }
    const std::optional<SwarmState> state = fresh_state(slot);
    if (state && state->ready) {
      continue;
    }
    if (missing.tellp() > 0) {
      missing << ", ";
    }
    missing << "slot " << slot;
    if (!neighbours_[static_cast<size_t>(slot)]) {
      missing << " (never heard from)";
    } else if (!state) {
      missing << " (link stale)";
    }
  }
  return missing.str();
}

void FormationDrone::run_flying()
{
  if (is_leader()) {
    run_leader();
  } else {
    run_follower();
  }
}

void FormationDrone::run_leader()
{
  if (!has_arrived()) {
    warn_if_leg_is_stalled();
    return;
  }

  ++route_index_;
  leg_started_ = now();

  if (route_index_ >= route_.size()) {
    RCLCPP_INFO(get_logger(), "ring complete, heading home. Swarm can follow.");
    // Order matters. done_ is what the next drone in the chain is watching
    // for, and it has to be true before we change phase, because the very
    // next thing that happens is a broadcast.
    done_ = true;
    phase_ = Phase::Returning;
    return;
  }

  const SwarmPoint & target = route_[route_index_];
  RCLCPP_INFO(
    get_logger(), "waypoint %zu of %zu: swarm frame north %.1f east %.1f alt %.1f",
    route_index_, route_.size() - 1U, target.north_m, target.east_m, -target.down_m);
}

void FormationDrone::run_follower()
{
  const int ahead = slot_ - 1;
  const std::optional<SwarmState> state = fresh_state(ahead);

  if (!state) {
    // Nothing recent enough to steer by. Hold the last target rather than
    // freezing the setpoint stream, because RULE 2 does not care why we
    // stopped having an opinion: pause the stream and PX4 drops offboard.
    const double age_s = link_age_s(ahead);
    if (age_s > LINK_LOST_ABORT_S) {
      RCLCPP_ERROR(
        get_logger(),
        "Nothing from slot %d for %.0f s. Abandoning the formation and going "
        "home, which is the same thing PX4 does when it loses its datalink.",
        ahead, age_s);
      done_ = true;
      phase_ = Phase::Returning;
      return;
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), PROBLEM_REPORT_INTERVAL_MS,
      "slot %d has gone quiet (%.1f s). Holding station rather than flying a "
      "formation off a position that is no longer true.", ahead, age_s);
    return;
  }

  // The drone ahead has finished. Peel off and go home, and our own broadcast
  // will say done, which is what releases the drone behind us. Nobody
  // coordinates this and there is no message type for "everybody land".
  if (state->done) {
    RCLCPP_INFO(get_logger(), "slot %d is done, heading home.", ahead);
    done_ = true;
    phase_ = Phase::Returning;
    return;
  }

  follower_target_ = chase_target(*state);
}

SwarmPoint FormationDrone::chase_target(const SwarmState & ahead) const
{
  const SwarmPoint me = my_swarm_position();
  const double ahead_north = static_cast<double>(ahead.north_m);
  const double ahead_east = static_cast<double>(ahead.east_m);

  // Which way is the drone ahead travelling? A position alone cannot say, and
  // a rule built on position alone can only ever hold a fixed RADIUS from it:
  // aim `follow_distance_m` back along the line to the leader and, whenever
  // that distance is already right, the target lands exactly on our own
  // position. We stop, the leader moves on, we chase, and the target collapses
  // onto us again. The follower slides around a circle instead of following,
  // which is the drone that hovers and jitters in place.
  //
  // So trail the leader's VELOCITY instead: aim at a point `follow_distance_m`
  // behind where it is heading. That is a direction to travel, not just a
  // distance to hold, and it converges.
  const std::optional<Neighbour> & entry =
    neighbours_.at(static_cast<size_t>(slot_ - 1));
  const std::optional<SwarmState> previous =
    entry ? entry->previous : std::nullopt;

  double heading_north = 0.0;
  double heading_east = 0.0;
  if (previous) {
    heading_north = ahead_north - static_cast<double>(previous->north_m);
    heading_east = ahead_east - static_cast<double>(previous->east_m);
  }
  const double heading_m = std::hypot(heading_north, heading_east);

  if (heading_m >= MIN_CHASE_BASELINE_M) {
    // It is moving. Sit behind it along its track.
    return SwarmPoint{
      ahead_north - follow_distance_m_ * heading_north / heading_m,
      ahead_east - follow_distance_m_ * heading_east / heading_m,
      -station_altitude_m()};
  }

  // It is not moving, or we have only ever seen one report from it. Fall back
  // to the line-of-sight rule, which is at least well defined when the leader
  // is stationary: hold `follow_distance_m` away from it on our own side.
  const double north_gap = ahead_north - me.north_m;
  const double east_gap = ahead_east - me.east_m;
  const double range_m = std::hypot(north_gap, east_gap);

  if (range_m < MIN_CHASE_BASELINE_M) {
    // Directly underneath it. The bearing is meaningless at this range and
    // would spin, so hold position and let the other drone move first.
    return SwarmPoint{me.north_m, me.east_m, -station_altitude_m()};
  }

  return SwarmPoint{
    ahead_north - follow_distance_m_ * north_gap / range_m,
    ahead_east - follow_distance_m_ * east_gap / range_m,
    -station_altitude_m()};
}

void FormationDrone::check_separation()
{
  int closest_slot = -1;
  double closest_m = std::numeric_limits<double>::infinity();

  const SwarmPoint me = my_swarm_position();
  for (int slot = 0; slot < count_; ++slot) {
    if (slot == slot_) {
      continue;
    }
    const std::optional<SwarmState> state = fresh_state(slot);
    if (!state) {
      continue;
    }
    const double north_error = static_cast<double>(state->north_m) - me.north_m;
    const double east_error = static_cast<double>(state->east_m) - me.east_m;
    const double down_error = static_cast<double>(state->down_m) - me.down_m;
    const double gap_m = std::sqrt(
      north_error * north_error + east_error * east_error + down_error * down_error);
    if (gap_m < closest_m) {
      closest_m = gap_m;
      closest_slot = slot;
    }
  }

  if (closest_slot < 0) {
    return;
  }
  if (closest_m < SEPARATION_WARNING_M) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), PROBLEM_REPORT_INTERVAL_MS,
      "slot %d is %.1f m away, inside the %.1f m we would like. Nothing in "
      "this node will do anything about that.",
      closest_slot, closest_m, SEPARATION_WARNING_M);
    return;
  }
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), SEPARATION_REPORT_INTERVAL_MS,
    "closest neighbour is slot %d at %.1f m.", closest_slot, closest_m);
}

void FormationDrone::run_arming()
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

std::string FormationDrone::describe_arming_blocker() const
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

void FormationDrone::warn_if_leg_is_stalled() const
{
  if ((now() - leg_started_).seconds() < LEG_STALL_WARNING_S) {
    return;
  }
  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), PROBLEM_REPORT_INTERVAL_MS,
    "still %.1f m from its target after %.0f s. nav_state %u, %s.",
    distance_to(to_local(current_target())), (now() - leg_started_).seconds(),
    vehicle_status_.value().nav_state,
    is_offboard() ? "still offboard" : "NOT in offboard mode any more");
}

void FormationDrone::warn_if_px4_took_control() const
{
  const bool airborne = phase_ == Phase::Climbing ||
    phase_ == Phase::WaitingForSwarm || phase_ == Phase::Flying ||
    phase_ == Phase::Returning;
  if (airborne && !is_offboard()) {
    const uint8_t nav_state = vehicle_status_.value().nav_state;
    // Name the mode rather than asserting a failsafe. PX4 leaves offboard for
    // two quite different reasons: a failsafe it decided on, or the offboard
    // control signal going stale, which lands it in AUTO_LOITER with
    // `failsafe` still clear. Saying "it is flying its own failsafe" for the
    // second case sends the reader looking for a failsafe that is not there.
    const char * reason = vehicle_status_.value().failsafe ?
      "a failsafe is active" :
      "the offboard setpoint stream went stale";
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), PROBLEM_REPORT_INTERVAL_MS,
      "PX4 left offboard mode (nav_state %u, %s). It is flying its own "
      "control now and this node is not in control.",
      static_cast<unsigned>(nav_state), reason);
  }
}

bool FormationDrone::is_armed() const
{
  return vehicle_status_.value().arming_state == VehicleStatus::ARMING_STATE_ARMED;
}

bool FormationDrone::is_offboard() const
{
  return vehicle_status_.value().nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD;
}

double FormationDrone::distance_to(const LocalPoint & target) const
{
  const VehicleLocalPosition & here = local_position_.value();
  const double north_error = static_cast<double>(here.x) - target.north_m;
  const double east_error = static_cast<double>(here.y) - target.east_m;
  const double down_error = static_cast<double>(here.z) - target.down_m;
  return std::sqrt(
    north_error * north_error + east_error * east_error + down_error * down_error);
}

bool FormationDrone::has_arrived() const
{
  return distance_to(to_local(current_target())) < ARRIVAL_RADIUS_M;
}

uint64_t FormationDrone::px4_timestamp_us() const
{
  // Zero, deliberately, rather than the ROS clock.
  //
  // PX4 stamps incoming uORB messages with hrt_absolute_time(), which is
  // microseconds since PX4 booted, not since the Unix epoch. The uXRCE-DDS
  // client bridges the two by subtracting a time_offset it derives from its
  // own time-sync handshake, and the generated deserialiser does:
  //
  //     if (topic.timestamp == 0) topic.timestamp = hrt_absolute_time();
  //     else topic.timestamp = min(topic.timestamp - time_offset, hrt_absolute_time());
  //
  // Sending the ROS clock therefore makes every message depend on that
  // handshake staying converged. When it does not -- PX4 logs "time jump
  // detected" and "time sync no longer converged" -- the subtraction yields
  // nonsense, the offboard_control_mode timestamp stops looking recent, and
  // PX4 declares offboard control signal lost and falls back to loiter. The
  // vehicle then sits on the ground until COM_DISARM_PRFLT disarms it.
  //
  // Zero takes the first branch, which needs no agreement about time at all.
  return 0;
}

void FormationDrone::publish_offboard_control_mode()
{
  OffboardControlMode message;
  message.position = true;
  message.timestamp = px4_timestamp_us();
  offboard_mode_publisher_->publish(message);
}

void FormationDrone::publish_setpoint(const LocalPoint & target)
{
  TrajectorySetpoint message;
  message.position = {
    static_cast<float>(target.north_m),
    static_cast<float>(target.east_m),
    static_cast<float>(target.down_m)};
  message.yaw = static_cast<float>(update_yaw_setpoint(target));
  message.timestamp = px4_timestamp_us();
  setpoint_publisher_->publish(message);
}

double FormationDrone::update_yaw_setpoint(const LocalPoint & target)
{
  if (!local_position_) {
    return yaw_setpoint_rad_.value_or(0.0);
  }
  const VehicleLocalPosition & here = local_position_.value();
  const double north_error = target.north_m - static_cast<double>(here.x);
  const double east_error = target.east_m - static_cast<double>(here.y);
  if (std::hypot(north_error, east_error) >= YAW_DEADBAND_M) {
    // atan2(east, north) because yaw in NED is measured from north towards
    // east, the opposite argument order to the usual maths one.
    yaw_setpoint_rad_ = std::atan2(east_error, north_error);
  }
  return yaw_setpoint_rad_.value_or(0.0);
}

void FormationDrone::send_vehicle_command(uint32_t command, float param1, float param2)
{
  VehicleCommand message;
  message.command = command;
  message.param1 = param1;
  message.param2 = param2;
  message.target_system = system_id_;
  message.target_component = PX4_COMPONENT_ID;
  message.source_system = system_id_;
  message.source_component = PX4_COMPONENT_ID;
  message.from_external = true;
  message.timestamp = px4_timestamp_us();
  command_publisher_->publish(message);
}

}  // namespace swarm_comms
