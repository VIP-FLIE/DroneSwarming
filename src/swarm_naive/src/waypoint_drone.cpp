// ===========================================================================
// waypoint_drone.cpp -- one drone in a swarm that does not know it is a swarm.
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

#include <cmath>
#include <cstdint>
#include <exception>
#include <memory>
#include <optional>
#include <stdexcept>
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
/// only heartbeats SITL instances 0-9, so drone 11 would never be able to arm.
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
rclcpp::QoS px4_stream_qos()
{
  rclcpp::QoS qos{rclcpp::KeepLast(1)};
  qos.best_effort();
  qos.transient_local();
  return qos;
}

/// QoS for /fmu/in/vehicle_command, which is the one exception. PX4's
/// uXRCE-DDS client serves vehicle_command as a DDS replier, and that replier
/// is RELIABLE. A BEST_EFFORT publisher is incompatible with it, so arm and
/// mode commands would be dropped in silence.
rclcpp::QoS px4_command_qos()
{
  rclcpp::QoS qos{rclcpp::KeepLast(1)};
  qos.reliable();
  qos.transient_local();
  return qos;
}

/// Turn a PX4 namespace into a topic prefix: "px4_1" becomes "/px4_1".
///
/// SITL instance 0 publishes on bare /fmu/..., exactly like a real aircraft's
/// autopilot, and instance N above 0 publishes on /px4_N/fmu/.... An empty
/// namespace therefore gives an empty prefix and the same binary serves both.
/// Verified in PX4 v1.16.1, ROMFS/px4fmu_common/init.d-posix/rcS line 299.
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

}  // namespace

using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;

// ===========================================================================
// The node. One instance of this flies one aircraft.
// ===========================================================================

class WaypointDrone : public rclcpp::Node
{
public:
  WaypointDrone()
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
    // wall-clock time stop agreeing. See docs/SIMULATION-FIDELITY.md.
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

  // -- Setting up ---------------------------------------------------------

  /// Refuse to construct on a parameter that cannot mean anything.
  ///
  /// Throwing here means the node never publishes a setpoint, which is the
  /// right response: a drone that takes off with a nonsense plan is worse than
  /// a drone that never leaves the ground.
  void validate_parameters() const
  {
    if (count_ < 1 || count_ > MAX_SWARM_SIZE) {
      throw std::invalid_argument(
              "count must be between 1 and " + std::to_string(MAX_SWARM_SIZE) +
              ". drone-gcs only heartbeats SITL instances 0-9, so a larger swarm "
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
  }

  /// The altitude this drone flies at, which is the only vertical deconfliction
  /// in the whole package. Drone 0 gets altitude_m_, drone 1 gets one layer
  /// above it, and so on.
  ///
  /// This is a guess baked in before takeoff. It works because the plan is
  /// known in advance and nothing ever deviates from it. Both halves of that
  /// sentence stop being true on real hardware.
  double station_altitude_m() const
  {
    return altitude_m_ + static_cast<double>(slot_) * layer_m_;
  }

  /// Vertex `index` of the ring, in the shared swarm frame.
  ///
  /// The ring has one vertex per drone, evenly spaced, measured clockwise from
  /// north so vertex 0 is due north of the centre. Indices past count_ wrap
  /// round on their own because cosine and sine are periodic, which is why
  /// build_route() can just keep counting upwards.
  SwarmPoint ring_vertex(int index, double down_m) const
  {
    const double angle_rad = 2.0 * M_PI * static_cast<double>(index) /
      static_cast<double>(count_);
    return SwarmPoint{
      ring_centre_north_m_ + radius_m_ * std::cos(angle_rad),
      ring_centre_east_m_ + radius_m_ * std::sin(angle_rad),
      down_m};
  }

  /// The whole flight, as a list of points in the shared swarm frame.
  ///
  /// Written in the shared frame on purpose. Nothing in here mentions where
  /// this particular drone's pad is except the first and last entries, so the
  /// ring is genuinely the same object for every drone in the swarm.
  std::vector<SwarmPoint> build_route() const
  {
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

  void announce_plan(const std::string & prefix) const
  {
    RCLCPP_INFO(
      get_logger(),
      "slot %d of %d. Pad at north %.1f east %.1f, flying at %.1f m, "
      "%zu waypoints round a %.1f m ring. Commanding %s/fmu/ as system %u.",
      slot_, count_, home_.north_m, home_.east_m, station_altitude_m(),
      route_.size(), radius_m_, prefix.empty() ? "" : prefix.c_str(),
      static_cast<unsigned>(system_id_));
  }

  // -- The control loop ---------------------------------------------------

  /// Runs CONTROL_LOOP_HZ times a second, for the whole flight.
  ///
  /// The order matters. Talking to PX4 comes first and happens unconditionally,
  /// because RULE 2 says that stream must never pause. Only then do we think
  /// about whether it is time to move on.
  void run_control_tick()
  {
    if (phase_ != Phase::Landing && phase_ != Phase::Done) {
      // Once PX4's landing routine has the vehicle our setpoints are no longer
      // wanted, so the stream stops there and not before.
      publish_offboard_control_mode();
      publish_setpoint(to_local(current_target()));
    }
    advance_phase();
  }

  /// Where this drone should be right now, in the shared swarm frame.
  ///
  /// Before the flight starts that is the first waypoint, which is the point
  /// directly above its own pad. Publishing a setpoint it is not at yet is
  /// harmless: PX4 is not acting on it until offboard is engaged, and RULE 2
  /// needs the stream to exist regardless.
  SwarmPoint current_target() const
  {
    return route_.at(phase_ == Phase::Flying ? route_index_ : 0U);
  }

  /// THE CONVERSION. This is the entire swarm problem, in two subtractions.
  ///
  /// The route is written in the shared swarm frame, whose origin is drone 0's
  /// pad. PX4 only understands this vehicle's own local frame, whose origin is
  /// this vehicle's own pad. Subtracting the pad offset moves a point from one
  /// to the other.
  ///
  /// Verified in the simulator: three vehicles spawned 6 m apart each report
  /// vehicle_local_position x and y within 3 cm of zero, so each really is
  /// measuring from its own pad and not from a shared world origin.
  ///
  /// down_m is passed through untouched, because every pad here is on the same
  /// flat ground and so shares an altitude datum. On sloping ground each drone
  /// would need a third offset for exactly the same reason it needs these two.
  LocalPoint to_local(const SwarmPoint & point) const
  {
    return LocalPoint{
      point.north_m - home_.north_m,
      point.east_m - home_.east_m,
      point.down_m};
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

  /// RULE 3: get the vehicle armed, and explain it if PX4 says no.
  ///
  /// A refusal here is almost always a preflight check doing its job. The one
  /// that catches everybody in a swarm is the datalink check: the x500 airframe
  /// sets NAV_DLL_ACT 2, "return home on datalink loss", and PX4 will not take
  /// off with a failsafe whose precondition is already broken. Each vehicle
  /// enforces it separately, so the classic symptom is drone 0 flying away
  /// while the rest sit on their pads. Supplying the link with drone-gcs is the
  /// fix; switching the failsafe off is not.
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

  /// Work down the route, one waypoint at a time, and land off the end of it.
  ///
  /// This is the entire flight logic of the package. Everything that makes it a
  /// swarm rather than a single drone happened before takeoff, in build_route()
  /// and to_local().
  void run_flying()
  {
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

  /// A leg that never finishes is nearly always PX4 having left offboard mode,
  /// so say which waypoint is stuck rather than sitting there silently.
  void warn_if_leg_is_stalled() const
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

  /// A failsafe pulls the vehicle out of offboard mode, and PX4 is right to win
  /// that argument. Say so plainly rather than streaming setpoints at a vehicle
  /// that has stopped listening.
  ///
  /// Killing drone-gcs mid-flight is how you watch this happen on purpose, and
  /// with a swarm you get to watch every drone react at once.
  void warn_if_px4_took_control() const
  {
    if (phase_ == Phase::Flying && !is_offboard()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), PROBLEM_REPORT_INTERVAL_MS,
        "PX4 left offboard mode (nav_state %u, failsafe %s). It is flying its own "
        "failsafe now and this node is not in control.",
        vehicle_status_.value().nav_state,
        vehicle_status_.value().failsafe ? "active" : "clear");
    }
  }

  /// Turns PX4's own view of why it will not arm into something you can act on.
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

  // -- Reading the vehicle ------------------------------------------------

  bool is_armed() const
  {
    return vehicle_status_.value().arming_state == VehicleStatus::ARMING_STATE_ARMED;
  }

  bool is_offboard() const
  {
    return vehicle_status_.value().nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  }

  /// Straight-line distance from where PX4 says we are to a point in our own
  /// local frame. Both sides have to be in the same frame, which is what the
  /// LocalPoint type is there to enforce.
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

  // -- Talking to PX4 -----------------------------------------------------

  uint64_t px4_timestamp_us() const
  {
    return static_cast<uint64_t>(get_clock()->now().nanoseconds() / NANOSECONDS_PER_MICROSECOND);
  }

  /// RULE 2, the stream itself.
  ///
  /// This message carries no position. It tells PX4 which FIELDS of the
  /// trajectory setpoint to act on, and its arrival is what PX4 uses to decide
  /// that an offboard controller is alive. It must go out on every tick, before
  /// the mode switch and after it, or offboard is refused or dropped. Fields we
  /// leave alone default to false, so PX4 acts on position only.
  void publish_offboard_control_mode()
  {
    OffboardControlMode message;
    message.position = true;
    message.timestamp = px4_timestamp_us();
    offboard_mode_publisher_->publish(message);
  }

  /// The actual "please be here" message, in metres, in THIS vehicle's local
  /// NED frame. By the time a point reaches here it has already been through
  /// to_local(), which is why the argument type is LocalPoint.
  ///
  /// Fields we do not set stay at zero, and PX4 reads zero as a real command
  /// rather than as "unset". That is what we want for velocity here, since each
  /// drone is meant to arrive at a waypoint and stop. To tell PX4 not to control
  /// something at all, set it to NaN.
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

  /// Point the nose along the direction of travel.
  ///
  /// Not required for the flight: PX4 will happily fly a quadrotor sideways,
  /// and hop.cpp leaves yaw at its default of zero, which commands "face
  /// north". Steering it is worth the six lines because a swarm that faces
  /// where it is going is far easier to read on screen, and because a real
  /// aircraft with a forward-facing camera or a fixed antenna has to do it.
  ///
  /// The deadband matters: the bearing to a point you are already sitting on is
  /// meaningless, so inside YAW_DEADBAND_M we hold the last commanded heading
  /// instead of chasing noise.
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
      // east, which is the opposite argument order to the usual maths one.
      yaw_setpoint_rad_ = std::atan2(east_error, north_error);
    }
    return yaw_setpoint_rad_.value_or(0.0);
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
    // Addressing, and the reason a swarm needs it. PX4 drops any command aimed
    // at a different system id, which is the only thing stopping this node from
    // arming every other drone on the field. SITL instance N answers to system
    // id N + 1 (PX4 v1.16.1, rcS line 140: `param set MAV_SYS_ID
    // $((px4_instance+1))`), so a wrong system_id looks like a node that runs
    // perfectly while nothing ever arms.
    message.target_system = system_id_;
    message.target_component = PX4_COMPONENT_ID;
    message.source_system = system_id_;
    message.source_component = PX4_COMPONENT_ID;
    message.from_external = true;   // "this came from a companion computer"
    message.timestamp = px4_timestamp_us();
    command_publisher_->publish(message);
  }

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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    // make_shared, and rclcpp owns it from here. There is no new or delete in
    // this file and nothing to free by hand.
    rclcpp::spin(std::make_shared<swarm_naive::WaypointDrone>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("waypoint_drone"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
