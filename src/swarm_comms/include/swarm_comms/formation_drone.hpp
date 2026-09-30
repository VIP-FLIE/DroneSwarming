// ===========================================================================
// formation_drone.hpp -- one drone in a swarm whose drones talk to each other.
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
// This file holds the declarations: the tunables, the shared helpers, the two
// frame types, the flight phases and the FormationDrone class. The method
// bodies are in formation_drone.cpp, and main() is in main.cpp.
//
// The PX4 handshake is repeated here verbatim rather than factored into a shared
// library, so that each package reads start to finish on its own. Strip the
// comments and normalise the names and 428 of waypoint_drone.cpp's 450 lines of
// code are in this package unchanged.
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
#pragma once

#include <cstdint>
#include <optional>
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

using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;
using swarm_comms::msg::SwarmState;

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

rclcpp::QoS px4_stream_qos();

rclcpp::QoS px4_command_qos();

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
rclcpp::QoS swarm_qos(int count);

std::string px4_topic_prefix(const std::string & px4_namespace);

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

// ===========================================================================
// The node. One instance of this flies one aircraft.
// ===========================================================================

class FormationDrone : public rclcpp::Node
{
public:
  FormationDrone();

private:
  /// One neighbour's last known state, and when WE received it.
  ///
  /// `received` is our own clock, not the sender's. Two drones do not share a
  /// clock, so comparing their stamp against ours would measure clock skew as
  /// well as age, and the difference would be indistinguishable from a delay.
  /// Measuring age by arrival time needs no agreement with anybody.
  ///
  /// `previous` is the report before this one, kept so a follower can work out
  /// which way the drone ahead is travelling. A position alone says where it
  /// is, not where it is going, and a pursuit rule built on position alone can
  /// only ever hold a fixed radius -- see chase_target().
  struct Neighbour
  {
    SwarmState state;
    rclcpp::Time received;
    std::optional<SwarmState> previous;
  };

  bool is_leader() const;

  // -- Setting up ---------------------------------------------------------

  void validate_parameters() const;

  /// The altitude this drone flies at. Unchanged from swarm_naive: the chase
  /// rule below is horizontal only, so every drone stays on its own layer and
  /// the layers are still what keeps non-adjacent drones apart.
  double station_altitude_m() const;

  /// Vertex `index` of the ring, in the shared swarm frame. One vertex per
  /// drone, evenly spaced, measured clockwise from north. Indices past count_
  /// wrap on their own because cosine and sine are periodic.
  SwarmPoint ring_vertex(int index, double down_m) const;

  /// The leader's route: once round the ring and back to where it joined.
  ///
  /// Shorter than swarm_naive's, because the climb and the trip home are phases
  /// of their own here rather than waypoints. They have to be: a follower has no
  /// route at all, so those two legs cannot live in one.
  std::vector<SwarmPoint> build_route() const;

  void announce_plan(const std::string & prefix) const;

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
  LocalPoint to_local(const SwarmPoint & point) const;

  /// This vehicle's own PX4 frame -> shared swarm frame. Used on everything we
  /// tell the other drones.
  ///
  /// Skipping this step is the classic swarm bug. Every drone would broadcast a
  /// position near zero, because every drone is near its own pad, and the whole
  /// formation would collapse onto one point while every number involved looked
  /// entirely plausible.
  SwarmPoint to_swarm(const LocalPoint & point) const;

  /// Where PX4 says we are, expressed in the shared frame.
  SwarmPoint my_swarm_position() const;

  // -- Talking to the other drones ----------------------------------------

  /// Called for every message on /swarm/state, including our own.
  void on_swarm_state(const SwarmState & message);

  /// Tell everybody where we are and how we are doing. Every tick, always.
  ///
  /// Published unconditionally once we have a position, including while still on
  /// the pad and including after landing. A drone that only broadcasts while it
  /// is flying is a drone the others cannot count at the barrier.
  void publish_swarm_state();

  /// How long since we last heard from a slot, in seconds. Infinity if never.
  double link_age_s(int slot) const;

  /// A neighbour's last report, if it is recent enough to steer by.
  std::optional<SwarmState> fresh_state(int slot) const;

  // -- The control loop ---------------------------------------------------

  /// Runs CONTROL_LOOP_HZ times a second, for the whole flight.
  ///
  /// Order matters, twice over. Talking to PX4 comes first and unconditionally,
  /// because RULE 2 says that stream must never pause. Talking to the swarm
  /// comes next and also unconditionally, for the same class of reason: a drone
  /// that stops broadcasting while it thinks about something has, as far as
  /// every other drone can tell, failed.
  void run_control_tick();

  /// Where this drone should be right now, in the shared swarm frame.
  ///
  /// The whole behavioural difference between the two packages is in this one
  /// function. swarm_naive returns an entry from a list fixed before takeoff.
  /// This returns, for every drone but the leader, a point computed from a
  /// message that arrived in the last tenth of a second.
  SwarmPoint current_target() const;

  /// Directly above this drone's own pad, at its own altitude.
  SwarmPoint station_point() const;

  void advance_phase();

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
  void run_waiting_for_swarm();

  /// Which slots have not reported ready, as something you can read.
  ///
  /// Empty means the barrier is satisfied. Our own readiness is known directly
  /// rather than read back off the topic, because on_swarm_state() drops our
  /// own echo.
  std::string slots_not_ready() const;

  /// The pattern. Two completely different jobs behind one phase.
  void run_flying();

  /// The leader flies the ring exactly the way swarm_naive does, and tells
  /// everybody when it is finished.
  void run_leader();

  /// A follower has no route. Its target is recomputed every tick from the last
  /// thing the drone ahead of it said.
  void run_follower();

  /// THE RULE THAT MAKES THIS A SWARM.
  ///
  /// Sit `follow_distance_m_` behind the drone ahead, measured back along the
  /// direction it is TRAVELLING. The direction comes from the difference
  /// between its last two reports, which is why Neighbour keeps a `previous`.
  ///
  /// Trailing the velocity rather than the line of sight is the whole of it.
  /// A rule that aims back along the line to the leader can only ever hold a
  /// fixed RADIUS from it: when that radius is already right, the target lands
  /// on the follower's own position, so it stops, the leader moves on, it
  /// chases, and the target collapses onto it again. The follower slides round
  /// a circle instead of following. Aiming behind the leader's track is a
  /// direction to travel rather than a distance to hold, and it converges.
  ///
  /// This target cannot be computed without a message from another aircraft,
  /// and it is recomputed ten times a second from live data. That is the whole
  /// difference between this package and swarm_naive.
  ///
  /// Horizontal only. Each drone stays on its own altitude layer, so the
  /// distance being held here is a ground distance and the vertical separation
  /// from the layers survives untouched.
  SwarmPoint chase_target(const SwarmState & ahead) const;

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
  void check_separation();

  // -- Talking to PX4. Identical to swarm_naive from here down. ------------

  /// RULE 3: get the vehicle armed, and explain it if PX4 says no. Each vehicle
  /// enforces the datalink failsafe separately, so the classic swarm symptom is
  /// drone 0 flying away while the rest sit on their pads.
  void run_arming();

  std::string describe_arming_blocker() const;

  void warn_if_leg_is_stalled() const;

  void warn_if_px4_took_control() const;

  bool is_armed() const;

  bool is_offboard() const;

  double distance_to(const LocalPoint & target) const;

  bool has_arrived() const;

  uint64_t px4_timestamp_us() const;

  /// RULE 2, the stream itself. Must go out on every tick, before the mode
  /// switch and after it, or offboard is refused or dropped.
  void publish_offboard_control_mode();

  void publish_setpoint(const LocalPoint & target);

  /// Point the nose along the direction of travel, with a deadband so the
  /// aircraft does not spin on the spot once it arrives.
  double update_yaw_setpoint(const LocalPoint & target);

  /// One MAVLink-style order to this vehicle's flight controller. PX4 drops any
  /// command aimed at a different system id, which is the only thing stopping
  /// this node from arming every other drone on the field. SITL instance N
  /// answers to system id N + 1 (PX4 v1.16.1, rcS line 140).
  void send_vehicle_command(uint32_t command, float param1 = 0.0F, float param2 = 0.0F);

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
