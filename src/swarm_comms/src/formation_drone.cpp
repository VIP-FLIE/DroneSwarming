// ===========================================================================
// formation_drone.cpp -- one drone in a swarm whose drones talk to each other.
//
// Same shape as swarm_naive/src/waypoint_drone.cpp: one process per vehicle,
// a slot number, a shared frame. The difference is that every drone now
// publishes where it is and reads where the others are, and uses that to decide
// where to go.
//
//     ros2 launch swarm_comms swarm.launch.py
//     ros2 launch swarm_comms swarm.launch.py count:=5 follow_distance_m:=6.0
//
// Read swarm_naive first. Then read this.
//
// The PX4 handshake is repeated here verbatim rather than factored into a shared
// library, so that each package reads start to finish on its own. Strip the
// comments and normalise the names and 428 of waypoint_drone.cpp's 450 lines of
// code are in this file unchanged.
//
// Diffing the two files is not worth it: the comments are rewritten throughout,
// so diff reports most of both. Everything communication adds is in six places,
// and they are the three sections below plus check_separation().
//
// THE FLIGHT
// ----------
// Everybody climbs off their own pad to their own altitude, and then WAITS.
// Nobody moves until every drone has reported that it is at its station. Once
// the barrier releases, drone 0 flies the ring from swarm_naive, and every
// other drone chases the one in front of it at a fixed distance. The result is
// a conga line that follows the leader round the ring, cutting the corners the
// way a towed thing does. When the leader finishes, it says so, and the swarm
// peels off and lands in order without anyone being told to.
//
// THE THREE THINGS COMMUNICATION MAKES POSSIBLE
// ---------------------------------------------
// 1. A SHARED CLOCK, in run_waiting_for_swarm(). Two drones do not share a
//    clock and cannot be made to cheaply, so this does not try. It uses a
//    barrier instead: nobody starts until everybody is ready. That converts
//    "start at the same time" into "start at the same EVENT", which needs no
//    clock at all. swarm_naive has nothing like it, which is why a drone that
//    took twenty seconds longer to arm there spends the whole flight in the
//    wrong part of the ring.
//
// 2. A POSITION THAT MEANS SOMETHING TO SOMEBODY ELSE, in to_swarm(). Each
//    drone converts out of its own PX4 frame before publishing, so the numbers
//    on the wire are in the shared swarm frame. Without that step every drone
//    would be broadcasting a position measured from its own pad, which nobody
//    else can use for anything.
//
// 3. A TARGET THAT IS A FUNCTION OF ANOTHER AIRCRAFT, in chase_target(). This
//    is the line that makes it a swarm rather than a fleet. The follower's
//    setpoint cannot be computed without a message from another drone, and it
//    is recomputed ten times a second from live data, so the formation repairs
//    itself when the leader does something unexpected.
//
// WHAT IT STILL DOES NOT DO
// -------------------------
// * It is not collision avoidance. check_separation() measures the gap to the
//   nearest neighbour and complains, and that is all it does. A real avoidance
//   layer would change the setpoint, and would have to handle two drones each
//   politely giving way into the other's new path.
// * The chase rule only constrains the drone in FRONT of you. Nothing stops
//   drone 3 from flying through where drone 1 happens to be. The altitude
//   layering inherited from swarm_naive is still doing real work here.
// * One radio, no redundancy, no acknowledgements. A drone whose link dies
//   holds station and then goes home, which is the safe answer and not a clever
//   one.
// ===========================================================================

#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>

// Our own message, generated from msg/SwarmState.msg by rosidl at build time.
// The header path is the message name in snake_case, under the package name.
#include <swarm_comms/msg/swarm_state.hpp>

namespace swarm_comms
{
namespace
{

// ---------------------------------------------------------------------------
// Tunables. The first block is identical to swarm_naive; the second is new.
// ---------------------------------------------------------------------------

constexpr double CONTROL_LOOP_HZ = 10.0;
constexpr int SETPOINTS_BEFORE_ARMING = 10;
constexpr double ARMING_TIMEOUT_S = 120.0;
constexpr int ARMING_ATTEMPT_INTERVAL_TICKS = 5;
constexpr double ARMING_QUIET_PERIOD_S = 5.0;
constexpr double ARRIVAL_RADIUS_M = 0.7;
constexpr double LEG_STALL_WARNING_S = 30.0;
constexpr double YAW_DEADBAND_M = 1.0;
constexpr int PROBLEM_REPORT_INTERVAL_MS = 5000;
constexpr int64_t NANOSECONDS_PER_MICROSECOND = 1000;
constexpr float BASE_MODE_CUSTOM_ENABLED = 1.0F;
constexpr float CUSTOM_MAIN_MODE_OFFBOARD = 6.0F;
constexpr uint8_t PX4_COMPONENT_ID = 1;
constexpr int MAX_SWARM_SIZE = 10;

// -- New in this package ----------------------------------------------------

/// The one topic every drone publishes to and subscribes to.
///
/// Outside any px4_ namespace on purpose. On a real aircraft the autopilot
/// occupies bare /fmu/..., so there is no namespace available to distinguish
/// drones and drone-to-drone traffic has to be identified by a field in the
/// message instead. Doing it that way here means the code does not change when
/// it leaves the simulator.
constexpr char SWARM_STATE_TOPIC[] = "/swarm/state";

/// How long a neighbour's report stays usable.
///
/// At 10 Hz this is ten missed messages in a row. Past it we stop steering by
/// that drone's position, because flying a formation off stale data is how two
/// aircraft end up in the same place: the number looks perfectly reasonable, it
/// is just no longer true.
constexpr double LINK_STALE_S = 1.0;

/// How long a neighbour can stay silent before we give up and go home.
///
/// Fifteen seconds of nothing is not a glitch. Holding station forever would
/// keep a drone airborne on a link that is not coming back, so this aborts the
/// pattern instead. Coming home on a lost link is also what PX4's own
/// datalink-loss failsafe does, for the same reason.
constexpr double LINK_LOST_ABORT_S = 15.0;

/// Below this the bearing to the drone ahead is noise rather than a direction,
/// so the follower holds station instead of chasing a number that is spinning.
constexpr double MIN_CHASE_BASELINE_M = 0.5;

/// Complain when two aircraft get closer than this. A monitor, not avoidance:
/// see check_separation().
constexpr double SEPARATION_WARNING_M = 2.5;

/// How often to report the closest neighbour while flying, in milliseconds.
constexpr int SEPARATION_REPORT_INTERVAL_MS = 5000;

/// How long to wait at the barrier before deciding the swarm is not coming.
///
/// Generous, because the last drone to boot may still be waiting on drone-gcs.
/// Past it the drone lands rather than hovering indefinitely on a swarm that
/// will never assemble.
constexpr double BARRIER_TIMEOUT_S = 180.0;

// ---------------------------------------------------------------------------
// RULE 1: QoS. See swarm_naive/src/waypoint_drone.cpp for the full explanation.
// PX4 publishes /fmu/out/ as BEST_EFFORT + TRANSIENT_LOCAL; a default ROS 2
// subscriber is RELIABLE + VOLATILE, which never matches and never fires.
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

/// QoS for /swarm/state, and the one QoS choice in this repo that is ours
/// rather than PX4's.
///
/// BEST_EFFORT, because this is telemetry. RELIABLE would retransmit a lost
/// position report, and a position that arrives late is worse than one that
/// never arrives: the formation would be steering by where a drone used to be
/// while a fresher report sat behind it in the queue. The message is designed
/// so that losing one costs nothing, which is what makes this safe. See
/// msg/SwarmState.msg.
///
/// VOLATILE, because a report from before you started running is by definition
/// out of date.
///
/// Depth is scaled with the size of the swarm. Every drone publishes to this
/// one topic, so `count` messages can arrive between two of our ticks; a depth
/// of 1 would mean hearing from one drone per tick and treating every other
/// drone as stale. Twice the swarm size leaves room for jitter.
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

// ---------------------------------------------------------------------------
// THE TWO FRAMES. Same pair of types as swarm_naive, and for the same reason:
// they hold the same three numbers and mean different things, so mixing them up
// should be a compile error rather than a crash site.
//
// swarm_naive only ever converts one way, swarm -> local, because its flight
// plan is fixed before takeoff. This package converts both ways: local ->
// swarm to publish where it is, and swarm -> local to fly where the others say
// it should go.
// ---------------------------------------------------------------------------

/// A point in the SHARED SWARM FRAME. Origin is drone 0's pad. No autopilot
/// anywhere knows this frame exists; it is a fiction agreed between the launch
/// file and every node, and it is the only frame in which a message from
/// another drone means anything.
struct SwarmPoint
{
  double north_m;
  double east_m;
  double down_m;   ///< NED: down is POSITIVE DOWNWARD, so 5 m up is -5.
};

/// A point in THIS VEHICLE'S OWN local NED frame: the frame PX4 speaks. Origin
/// is wherever this drone's position estimate started, i.e. its own pad.
struct LocalPoint
{
  double north_m;
  double east_m;
  double down_m;
};

}  // namespace

using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;
using swarm_comms::msg::SwarmState;

// ===========================================================================
// The node. One instance of this flies one aircraft.
// ===========================================================================

class FormationDrone : public rclcpp::Node
{
public:
  FormationDrone()
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

private:
  /// The stages of the flight, in the order they happen.
  ///
  /// Priming, Arming and EngagingOffboard are PX4's required order and are
  /// identical to swarm_naive. WaitingForSwarm is the new one, and it is the
  /// entire shared-clock mechanism.
  enum class Phase
  {
    Priming,           ///< building up the setpoint stream PX4 insists on
    Arming,            ///< asking PX4 to arm, repeatedly
    EngagingOffboard,  ///< asking PX4 to hand control to our setpoints
    Climbing,          ///< straight up off its own pad to its own altitude
    WaitingForSwarm,   ///< at station, holding, until everyone else is too
    Flying,            ///< leader flies the ring, everyone else chases
    Returning,         ///< back over its own pad
    Landing,           ///< PX4's own landing routine has the controls
    Done               ///< on the ground, disarmed, nothing left to do
  };

  /// One neighbour's last known state, and when WE received it.
  ///
  /// `received` is our own clock, not the sender's. Two drones do not share a
  /// clock, so comparing their stamp against ours would measure clock skew as
  /// well as age, and the difference would be indistinguishable from a delay.
  /// Measuring age by arrival time needs no agreement with anybody.
  struct Neighbour
  {
    SwarmState state;
    rclcpp::Time received;
  };

  bool is_leader() const {return slot_ == 0;}

  // -- Setting up ---------------------------------------------------------

  void validate_parameters() const
  {
    if (count_ < 1 || count_ > MAX_SWARM_SIZE) {
      throw std::invalid_argument(
              "count must be between 1 and " + std::to_string(MAX_SWARM_SIZE) +
              ". drone-gcs only heartbeats instances "
              "instance_base + 1 to instance_base + 10, so a larger swarm "
              "would have vehicles that can never arm.");
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

  /// The altitude this drone flies at. Unchanged from swarm_naive: the chase
  /// rule below is horizontal only, so every drone stays on its own layer and
  /// the layers are still what keeps non-adjacent drones apart.
  double station_altitude_m() const
  {
    return altitude_m_ + static_cast<double>(slot_) * layer_m_;
  }

  /// Vertex `index` of the ring, in the shared swarm frame. One vertex per
  /// drone, evenly spaced, measured clockwise from north. Indices past count_
  /// wrap on their own because cosine and sine are periodic.
  SwarmPoint ring_vertex(int index, double down_m) const
  {
    const double angle_rad = 2.0 * M_PI * static_cast<double>(index) /
      static_cast<double>(count_);
    return SwarmPoint{
      ring_centre_north_m_ + radius_m_ * std::cos(angle_rad),
      ring_centre_east_m_ + radius_m_ * std::sin(angle_rad),
      down_m};
  }

  /// The leader's route: once round the ring and back to where it joined.
  ///
  /// Shorter than swarm_naive's, because the climb and the trip home are phases
  /// of their own here rather than waypoints. They have to be: a follower has no
  /// route at all, so those two legs cannot live in one.
  std::vector<SwarmPoint> build_route() const
  {
    const double cruise_down_m = -station_altitude_m();
    std::vector<SwarmPoint> route;
    const int vertices_to_visit = count_ * laps_;
    for (int step = 0; step <= vertices_to_visit; ++step) {
      route.push_back(ring_vertex(step, cruise_down_m));
    }
    return route;
  }

  void announce_plan(const std::string & prefix) const
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

  // -- The two frames -----------------------------------------------------
  //
  // These are exact inverses and live next to each other on purpose. Getting
  // one of them backwards is the bug that makes a formation mirror itself.

  /// Shared swarm frame -> this vehicle's own PX4 frame. Used on everything we
  /// hand to PX4.
  ///
  /// down_m passes through untouched because every pad is on the same flat
  /// ground and so shares an altitude datum. On sloping ground each drone would
  /// need a third offset for the same reason it needs these two.
  LocalPoint to_local(const SwarmPoint & point) const
  {
    return LocalPoint{
      point.north_m - home_.north_m,
      point.east_m - home_.east_m,
      point.down_m};
  }

  /// This vehicle's own PX4 frame -> shared swarm frame. Used on everything we
  /// tell the other drones.
  ///
  /// Skipping this step is the classic swarm bug. Every drone would broadcast a
  /// position near zero, because every drone is near its own pad, and the whole
  /// formation would collapse onto one point while every number involved looked
  /// entirely plausible.
  SwarmPoint to_swarm(const LocalPoint & point) const
  {
    return SwarmPoint{
      point.north_m + home_.north_m,
      point.east_m + home_.east_m,
      point.down_m};
  }

  /// Where PX4 says we are, expressed in the shared frame.
  SwarmPoint my_swarm_position() const
  {
    const VehicleLocalPosition & here = local_position_.value();
    return to_swarm(
      LocalPoint{
        static_cast<double>(here.x),
        static_cast<double>(here.y),
        static_cast<double>(here.z)});
  }

  // -- Talking to the other drones ----------------------------------------

  /// Called for every message on /swarm/state, including our own.
  void on_swarm_state(const SwarmState & message)
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
    neighbours_[static_cast<size_t>(message.slot)] = Neighbour{message, now()};
  }

  /// Tell everybody where we are and how we are doing. Every tick, always.
  ///
  /// Published unconditionally once we have a position, including while still on
  /// the pad and including after landing. A drone that only broadcasts while it
  /// is flying is a drone the others cannot count at the barrier.
  void publish_swarm_state()
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

  /// How long since we last heard from a slot, in seconds. Infinity if never.
  double link_age_s(int slot) const
  {
    const auto & entry = neighbours_.at(static_cast<size_t>(slot));
    if (!entry) {
      return std::numeric_limits<double>::infinity();
    }
    return (now() - entry->received).seconds();
  }

  /// A neighbour's last report, if it is recent enough to steer by.
  std::optional<SwarmState> fresh_state(int slot) const
  {
    if (link_age_s(slot) > LINK_STALE_S) {
      return std::nullopt;
    }
    return neighbours_.at(static_cast<size_t>(slot))->state;
  }

  // -- The control loop ---------------------------------------------------

  /// Runs CONTROL_LOOP_HZ times a second, for the whole flight.
  ///
  /// Order matters, twice over. Talking to PX4 comes first and unconditionally,
  /// because RULE 2 says that stream must never pause. Talking to the swarm
  /// comes next and also unconditionally, for the same class of reason: a drone
  /// that stops broadcasting while it thinks about something has, as far as
  /// every other drone can tell, failed.
  void run_control_tick()
  {
    if (phase_ != Phase::Landing && phase_ != Phase::Done) {
      publish_offboard_control_mode();
      publish_setpoint(to_local(current_target()));
    }
    publish_swarm_state();
    advance_phase();
  }

  /// Where this drone should be right now, in the shared swarm frame.
  ///
  /// The whole behavioural difference between the two packages is in this one
  /// function. swarm_naive returns an entry from a list fixed before takeoff.
  /// This returns, for every drone but the leader, a point computed from a
  /// message that arrived in the last tenth of a second.
  SwarmPoint current_target() const
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

  /// Directly above this drone's own pad, at its own altitude.
  SwarmPoint station_point() const
  {
    return SwarmPoint{home_.north_m, home_.east_m, -station_altitude_m()};
  }

  void advance_phase()
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

  /// THE SHARED CLOCK, solved without a clock.
  ///
  /// Hold station until every drone reports ready. That turns "start at the
  /// same time", which needs synchronised clocks and is genuinely hard over a
  /// radio, into "start at the same event", which needs one bit per drone.
  ///
  /// It is not free: the swarm now moves at the speed of its slowest member,
  /// and a drone that never becomes ready stops everybody. The timeout below is
  /// the answer to the second half of that, and naming the missing slots is the
  /// difference between a bug you can fix and a swarm that just sits there.
  void run_waiting_for_swarm()
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

  /// Which slots have not reported ready, as something you can read.
  ///
  /// Empty means the barrier is satisfied. Our own readiness is known directly
  /// rather than read back off the topic, because on_swarm_state() drops our
  /// own echo.
  std::string slots_not_ready() const
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

  /// The pattern. Two completely different jobs behind one phase.
  void run_flying()
  {
    if (is_leader()) {
      run_leader();
    } else {
      run_follower();
    }
  }

  /// The leader flies the ring exactly the way swarm_naive does, and tells
  /// everybody when it is finished.
  void run_leader()
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

  /// A follower has no route. Its target is recomputed every tick from the last
  /// thing the drone ahead of it said.
  void run_follower()
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

  /// THE RULE THAT MAKES THIS A SWARM.
  ///
  /// Sit `follow_distance_m_` behind the drone ahead, on the line from us to
  /// it. One vector subtraction, one normalisation, and the chain arranges
  /// itself: whatever the leader does, each follower converges on a point a
  /// fixed distance back along its own line of sight, so the formation bends
  /// round corners instead of breaking.
  ///
  /// This target cannot be computed without a message from another aircraft,
  /// and it is recomputed ten times a second from live data. That is the whole
  /// difference between this package and swarm_naive, and it is four lines.
  ///
  /// Horizontal only. Each drone stays on its own altitude layer, so the
  /// distance being held here is a ground distance and the vertical separation
  /// from the layers survives untouched.
  SwarmPoint chase_target(const SwarmState & ahead) const
  {
    const SwarmPoint me = my_swarm_position();
    const double north_gap = static_cast<double>(ahead.north_m) - me.north_m;
    const double east_gap = static_cast<double>(ahead.east_m) - me.east_m;
    const double range_m = std::hypot(north_gap, east_gap);

    if (range_m < MIN_CHASE_BASELINE_M) {
      // Directly underneath it. The bearing is meaningless at this range and
      // would spin, so hold position and let the other drone move first.
      return SwarmPoint{me.north_m, me.east_m, -station_altitude_m()};
    }

    // Note the sign: we aim at a point BEHIND the drone ahead, measured back
    // along the line towards us. If we are closer than follow_distance_m_ the
    // target lands behind us and we back off, which is what stops the chain
    // from concertinaing when the leader slows down.
    return SwarmPoint{
      static_cast<double>(ahead.north_m) - follow_distance_m_ * north_gap / range_m,
      static_cast<double>(ahead.east_m) - follow_distance_m_ * east_gap / range_m,
      -station_altitude_m()};
  }

  /// Measure the gap to the nearest other aircraft and report it.
  ///
  /// A MONITOR, NOT AVOIDANCE. It changes no setpoint and moves no drone. It is
  /// here because it is the number that tells you whether the formation is
  /// working, and because it is exactly the measurement swarm_naive cannot make
  /// at all: without the messages, no drone has any idea where any other drone
  /// is.
  ///
  /// Real avoidance starts here and then has to answer the hard part, which is
  /// that two aircraft both politely giving way can give way into each other.
  void check_separation()
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
      const double gap_m = std::sqrt(
        std::pow(static_cast<double>(state->north_m) - me.north_m, 2) +
        std::pow(static_cast<double>(state->east_m) - me.east_m, 2) +
        std::pow(static_cast<double>(state->down_m) - me.down_m, 2));
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

  // -- Talking to PX4. Identical to swarm_naive from here down. ------------

  /// RULE 3: get the vehicle armed, and explain it if PX4 says no. Each vehicle
  /// enforces the datalink failsafe separately, so the classic swarm symptom is
  /// drone 0 flying away while the rest sit on their pads.
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

  std::string describe_arming_blocker() const
  {
    const VehicleStatus & status = vehicle_status_.value();
    if (status.gcs_connection_lost) {
      return "PX4 reports no ground control station link, so its datalink-loss "
             "failsafe is blocking arming. Is drone-gcs running, and does it "
             "report one linked system per drone?";
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

  void warn_if_leg_is_stalled() const
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

  void warn_if_px4_took_control() const
  {
    const bool airborne = phase_ == Phase::Climbing ||
      phase_ == Phase::WaitingForSwarm || phase_ == Phase::Flying ||
      phase_ == Phase::Returning;
    if (airborne && !is_offboard()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), PROBLEM_REPORT_INTERVAL_MS,
        "PX4 left offboard mode (nav_state %u, failsafe %s). It is flying its own "
        "failsafe now and this node is not in control.",
        vehicle_status_.value().nav_state,
        vehicle_status_.value().failsafe ? "active" : "clear");
    }
  }

  bool is_armed() const
  {
    return vehicle_status_.value().arming_state == VehicleStatus::ARMING_STATE_ARMED;
  }

  bool is_offboard() const
  {
    return vehicle_status_.value().nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  }

  double distance_to(const LocalPoint & target) const
  {
    const VehicleLocalPosition & here = local_position_.value();
    const double north_error = static_cast<double>(here.x) - target.north_m;
    const double east_error = static_cast<double>(here.y) - target.east_m;
    const double down_error = static_cast<double>(here.z) - target.down_m;
    return std::sqrt(
      north_error * north_error + east_error * east_error + down_error * down_error);
  }

  bool has_arrived() const
  {
    return distance_to(to_local(current_target())) < ARRIVAL_RADIUS_M;
  }

  uint64_t px4_timestamp_us() const
  {
    return static_cast<uint64_t>(get_clock()->now().nanoseconds() / NANOSECONDS_PER_MICROSECOND);
  }

  /// RULE 2, the stream itself. Must go out on every tick, before the mode
  /// switch and after it, or offboard is refused or dropped.
  void publish_offboard_control_mode()
  {
    OffboardControlMode message;
    message.position = true;
    message.timestamp = px4_timestamp_us();
    offboard_mode_publisher_->publish(message);
  }

  void publish_setpoint(const LocalPoint & target)
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

  /// Point the nose along the direction of travel, with a deadband so the
  /// aircraft does not spin on the spot once it arrives.
  double update_yaw_setpoint(const LocalPoint & target)
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

  /// One MAVLink-style order to this vehicle's flight controller. PX4 drops any
  /// command aimed at a different system id, which is the only thing stopping
  /// this node from arming every other drone on the field. SITL instance N
  /// answers to system id N + 1 (PX4 v1.16.1, rcS line 140).
  void send_vehicle_command(uint32_t command, float param1 = 0.0F, float param2 = 0.0F)
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

  // -- State --------------------------------------------------------------

  // Identity, fixed at launch.
  int slot_{0};
  int count_{3};
  uint8_t system_id_{1};
  SwarmPoint home_{0.0, 0.0, 0.0};

  // The shared flight plan, fixed at launch. Only the leader has a route_.
  double ring_centre_north_m_{10.0};
  double ring_centre_east_m_{0.0};
  double radius_m_{8.0};
  double altitude_m_{4.0};
  double layer_m_{2.0};
  int laps_{1};
  double follow_distance_m_{5.0};
  std::vector<SwarmPoint> route_;

  // Where we are in the flight.
  Phase phase_{Phase::Priming};
  std::size_t route_index_{0};
  int priming_ticks_{0};
  int arming_attempts_{0};
  rclcpp::Time arming_started_;
  rclcpp::Time barrier_started_;
  rclcpp::Time leg_started_;
  std::optional<double> yaw_setpoint_rad_;

  // What we tell the swarm about ourselves. Both are levels, not events: they
  // go out in every message for as long as they are true.
  bool at_station_{false};
  bool done_{false};

  // What the swarm tells us. Indexed by slot; empty means never heard from.
  std::vector<std::optional<Neighbour>> neighbours_;
  SwarmPoint follower_target_{0.0, 0.0, 0.0};

  std::optional<VehicleLocalPosition> local_position_;
  std::optional<VehicleStatus> vehicle_status_;

  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_mode_publisher_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr setpoint_publisher_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr command_publisher_;
  rclcpp::Publisher<SwarmState>::SharedPtr swarm_publisher_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_subscription_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_subscription_;
  rclcpp::Subscription<SwarmState>::SharedPtr swarm_subscription_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace swarm_comms

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<swarm_comms::FormationDrone>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("formation_drone"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
