// Copyright 2026 DroneSwarming maintainers.
#include "mission_execution/mission_execution_node.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <future>
#include <limits>
#include <optional>
#include <regex>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include "mission_execution/geometry.hpp"

namespace mission_execution
{
namespace
{

constexpr std::chrono::milliseconds callback_poll_interval(100);
constexpr double minimum_child_result_timeout_sec = 60.0;
constexpr std::chrono::seconds cancellation_grace_period(15);
constexpr double result_timeout_margin_sec = 30.0;
constexpr double minimum_look_at_distance_m = 0.1;
constexpr float minimum_orbit_radius_m = 1.0F;
constexpr std::size_t minimum_trajectory_points = 2;
constexpr std::size_t maximum_plan_json_bytes = 65536;
constexpr std::size_t maximum_planned_waypoints = 256;
constexpr float maximum_planned_position_tolerance_m = 10.0F;
constexpr float maximum_planned_orbit_speed_mps = 10.0F;
constexpr double maximum_planned_orbit_acceleration_mps2 = 2.0;
constexpr float maximum_planned_orbit_timeout_sec = 600.0F;
constexpr double hard_maximum_plan_altitude_m = 120.0;
constexpr double hard_maximum_plan_distance_m = 1000.0;

/// Stores synchronization state for one asynchronous child action.
template<typename ActionT>
struct ChildActionState
{
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;
  using WrappedResult = typename GoalHandle::WrappedResult;

  std::mutex mutex;
  std::condition_variable condition;
  std::optional<WrappedResult> result;
  typename GoalHandle::SharedPtr accepted_goal;
  bool cancel_on_response{false};
};

/// Reports child success and whether parent cancellation can be confirmed safely.
struct ChildActionOutcome
{
  bool succeeded{false};
  std::string message;
  bool cancellation_completed{true};
};

/// Runs a child action and forwards its feedback.
/// Propagates parent cancellation and waits for the child's terminal result.
template<typename ActionT, typename GoalT, typename FeedbackCallback>
ChildActionOutcome run_child_action(
  const typename rclcpp_action::Client<ActionT>::SharedPtr & client,
  const std::shared_ptr<std::atomic<bool>> & child_pending,
  const GoalT & goal,
  const std::chrono::steady_clock::time_point & response_deadline,
  const std::chrono::steady_clock::time_point & result_deadline,
  const std::function<bool()> & parent_is_canceling,
  const std::function<std::string()> & validate_state,
  FeedbackCallback feedback_callback,
  const std::function<void(const typename ActionT::Result &)> & copy_result =
  [](const typename ActionT::Result &) {},
  const std::function<std::string()> & execution_error = []() {return std::string();})
{
  using Client = rclcpp_action::Client<ActionT>;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;
  using WrappedResult = typename GoalHandle::WrappedResult;
  using Feedback = typename ActionT::Feedback;

  while (std::chrono::steady_clock::now() < response_deadline && rclcpp::ok()) {
    if (parent_is_canceling()) {
      return {false, "Command interrupted before starting the child action"};
    }
    if (!child_pending->load() && client->wait_for_action_server(callback_poll_interval)) {
      break;
    }
    std::this_thread::sleep_for(callback_poll_interval);
  }
  if (parent_is_canceling()) {
    return {false, "Command interrupted before starting the child action"};
  }
  if (child_pending->load()) {
    return {false, "Previous vehicle operation has no confirmed terminal result", false};
  }
  const std::string initial_execution_error = execution_error();
  if (!initial_execution_error.empty()) {
    return {false, initial_execution_error, false};
  }
  if (!client->action_server_is_ready()) {
    return {false, "Vehicle-control action server is unavailable"};
  }

  const std::string state_failure = validate_state();
  if (!state_failure.empty()) {
    return {false, state_failure};
  }
  std::shared_ptr<ChildActionState<ActionT>> child_state =
    std::make_shared<ChildActionState<ActionT>>();
  const std::weak_ptr<Client> weak_client = client;
  typename Client::SendGoalOptions options;
  options.feedback_callback = [feedback_callback](
      typename GoalHandle::SharedPtr,
      const std::shared_ptr<const Feedback> feedback) {
        feedback_callback(*feedback);
      };
  options.goal_response_callback = [child_state, child_pending, weak_client](
      typename GoalHandle::SharedPtr accepted_goal) {
      bool should_cancel = false;
      {
        std::lock_guard<std::mutex> lock(child_state->mutex);
        child_state->accepted_goal = accepted_goal;
        should_cancel = child_state->cancel_on_response;
      }
      if (!accepted_goal) {
        child_pending->store(false);
      } else if (should_cancel) {
        const std::shared_ptr<Client> active_client = weak_client.lock();
        if (active_client && rclcpp::ok()) {
          active_client->async_cancel_goal(accepted_goal);
        }
      }
    };
  options.result_callback = [child_state, child_pending](const WrappedResult & result) {
      {
        std::lock_guard<std::mutex> lock(child_state->mutex);
        child_state->result = result;
        child_pending->store(false);
      }
      child_state->condition.notify_all();
    };
  child_pending->store(true);
  std::shared_future<typename GoalHandle::SharedPtr> goal_response =
    client->async_send_goal(goal, options);

  while (goal_response.wait_for(callback_poll_interval) != std::future_status::ready) {
    if (std::chrono::steady_clock::now() >= response_deadline || !rclcpp::ok()) {
      typename GoalHandle::SharedPtr late_goal;
      {
        std::lock_guard<std::mutex> lock(child_state->mutex);
        child_state->cancel_on_response = true;
        late_goal = child_state->accepted_goal;
      }
      if (late_goal) {
        client->async_cancel_goal(late_goal);
      }
      return {false, "Timed out waiting for the child goal response; late acceptance will be canceled", false};
    }
  }
  typename GoalHandle::SharedPtr child_goal = goal_response.get();
  if (!child_goal) {
    return {false, "Vehicle-control rejected the child action goal"};
  }

  bool cancellation_sent = false;
  bool child_timed_out = false;
  std::string execution_failure;
  std::chrono::steady_clock::time_point cancel_deadline = result_deadline;
  for (;;) {
    const std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    if (!cancellation_sent) {
      execution_failure = execution_error();
    }
    if (!cancellation_sent &&
      (parent_is_canceling() || !execution_failure.empty() || now >= result_deadline))
    {
      client->async_cancel_goal(child_goal);
      cancellation_sent = true;
      child_timed_out = now >= result_deadline;
      // PX4 landing can reject cancellation after committing to descent.
      cancel_deadline = std::is_same_v<ActionT, swarm_interfaces::action::Land> ?
        std::max(result_deadline, now + cancellation_grace_period) :
        now + cancellation_grace_period;
    }

    {
      std::unique_lock<std::mutex> lock(child_state->mutex);
      if (child_state->result) {
        const WrappedResult result = *child_state->result;
        if (result.result) {
          copy_result(*result.result);
        }
        if (!execution_failure.empty()) {
          return {false, execution_failure, false};
        }
        if (child_timed_out) {
          return {false, "Child action exceeded its result deadline", false};
        }
        if (result.code == rclcpp_action::ResultCode::CANCELED) {
          return {false, result.result && !result.result->message.empty() ?
            result.result->message : "Active child action was canceled"};
        }
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
          result.result->success)
        {
          if (parent_is_canceling()) {
            return {false, "Command interrupted; active child action finished"};
          }
          return {true, result.result->message};
        }
        return {false, result.result ? result.result->message :
          "Child action returned no result", false};
      }
      child_state->condition.wait_for(lock, callback_poll_interval);
    }

    if (cancellation_sent && std::chrono::steady_clock::now() >= cancel_deadline) {
      return {false, "Child cancellation has no confirmed terminal result; further dispatch is blocked", false};
    }
    if (!rclcpp::ok()) {
      return {false, "ROS shutdown interrupted the command", false};
    }
  }
}

/// Ends unsuccessful requests with a cancellation status only when the caller canceled.
template<typename ActionT>
void finish_unsuccessful(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> & handle,
  const std::shared_ptr<typename ActionT::Result> & result, const bool cancellation_completed = true)
{
  result->success = false;
  if (handle->is_canceling() && cancellation_completed) {
    handle->canceled(result);
  } else {
    handle->abort(result);
  }
}

/// Rejects non-finite or negative durations before reserving command authority.
bool is_valid_timeout(const float timeout_sec)
{
  return std::isfinite(timeout_sec) && timeout_sec >= 0.0F;
}

/// Checks application positions at the command boundary.
bool is_finite_point(const geometry_msgs::msg::Point & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

/// Validates public operation fields; vehicle control applies its execution limits again.
bool is_valid_goal(const swarm_interfaces::action::Takeoff::Goal & goal)
{
  return std::isfinite(goal.altitude_m) && goal.altitude_m > 0.0F;
}

/// Validates the absolute navigation contract and optional look-at position.
bool is_valid_goal(const swarm_interfaces::action::NavigateTo::Goal & goal)
{
  return goal.frame_id == "local_enu" && is_finite_point(goal.target_position) &&
         is_valid_timeout(goal.position_tolerance_m) && is_valid_timeout(goal.timeout_sec) &&
         (!goal.look_at_target || (is_finite_point(goal.look_at_position) &&
         std::hypot(goal.look_at_position.x - goal.target_position.x,
         goal.look_at_position.y - goal.target_position.y) > minimum_look_at_distance_m));
}

/// Validates trajectory time order, positions, and requested heading fields.
bool is_valid_goal(const swarm_interfaces::action::FollowTrajectory::Goal & goal)
{
  if (goal.points.size() < minimum_trajectory_points || goal.points.front().time_from_start_sec != 0.0F ||
    !is_valid_timeout(goal.timeout_sec) || !is_valid_timeout(goal.position_tolerance_m) ||
    !is_valid_timeout(goal.yaw_tolerance_rad) ||
    (!goal.control_yaw && goal.yaw_tolerance_rad != 0.0F))
  {
    return false;
  }
  float previous_time_sec = -1.0F;
  for (const swarm_interfaces::msg::TrajectoryPoint & point : goal.points) {
    if (!is_finite_point(point.position) || !std::isfinite(point.time_from_start_sec) ||
      point.time_from_start_sec <= previous_time_sec ||
      (goal.control_yaw && !std::isfinite(point.yaw_rad)))
    {
      return false;
    }
    previous_time_sec = point.time_from_start_sec;
  }
  return true;
}

/// Validates an orbit's application frame, geometry, and positive duration.
bool is_valid_goal(const swarm_interfaces::action::OrbitPoint::Goal & goal)
{
  return goal.frame_id == "local_enu" && is_finite_point(goal.center_position) &&
         std::isfinite(goal.radius_m) && goal.radius_m >= minimum_orbit_radius_m &&
         std::isfinite(goal.tangential_speed_mps) && goal.tangential_speed_mps > 0.0F &&
         std::isfinite(goal.revolutions) && goal.revolutions > 0.0F &&
         is_valid_timeout(goal.yaw_tolerance_rad) &&
         (goal.face_center || goal.yaw_tolerance_rad == 0.0F) &&
         std::isfinite(goal.timeout_sec) && goal.timeout_sec > 0.0F;
}

/// Rejects invalid landing requests before they can interrupt ordinary flight.
bool is_valid_goal(const swarm_interfaces::action::Land::Goal & goal)
{
  return is_valid_timeout(goal.timeout_sec);
}

/// Builds a steady-clock deadline duration_sec into the future.
std::chrono::steady_clock::time_point deadline_after(const double duration_sec)
{
  const std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
  const std::chrono::duration<double> remaining = std::chrono::steady_clock::time_point::max() - now;
  if (duration_sec >= remaining.count()) {
    return std::chrono::steady_clock::time_point::max();
  }
  return now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(duration_sec));
}

}  // namespace

MissionExecutionNode::MissionExecutionNode(const rclcpp::NodeOptions & options)
: Node("mission_execution", options)
{
  telemetry_timeout_sec_ = declare_parameter<double>("telemetry_timeout_sec", 1.0);
  starting_position_timeout_sec_ = declare_parameter<double>("starting_position_timeout_sec", 10.0);
  action_server_timeout_sec_ = declare_parameter<double>("action_server_timeout_sec", 10.0);
  child_result_timeout_sec_ = declare_parameter<double>("child_result_timeout_sec", 90.0);
  default_waypoint_timeout_sec_ = declare_parameter<double>("default_waypoint_timeout_sec", 30.0);
  maximum_plan_altitude_m_ = declare_parameter<double>("maximum_plan_altitude_m", 30.0);
  maximum_plan_distance_m_ = declare_parameter<double>("maximum_plan_distance_m", 100.0);
  vehicle_id_ = declare_parameter<std::string>("vehicle_id", "");
  if (vehicle_id_.empty()) {
    vehicle_id_ = get_namespace() == std::string("/") ? "uav_1" :
      std::string(get_namespace()).substr(1);
    std::replace(vehicle_id_.begin(), vehicle_id_.end(), '/', '_');
  }
  if (!std::regex_match(vehicle_id_, std::regex("[A-Za-z][A-Za-z0-9_-]{0,63}"))) {
    throw std::invalid_argument("vehicle_id must be an ASCII letter followed by up to 63 ID characters");
  }
  if (!std::isfinite(maximum_plan_altitude_m_) || maximum_plan_altitude_m_ <= 0.0 ||
    maximum_plan_altitude_m_ > hard_maximum_plan_altitude_m ||
    !std::isfinite(maximum_plan_distance_m_) || maximum_plan_distance_m_ <= 0.0 ||
    maximum_plan_distance_m_ > hard_maximum_plan_distance_m)
  {
    throw std::invalid_argument("JSON plan bounds exceed supported altitude or distance limits");
  }
  if (!std::isfinite(telemetry_timeout_sec_) || telemetry_timeout_sec_ <= 0.0 ||
    !std::isfinite(starting_position_timeout_sec_) || starting_position_timeout_sec_ <= 0.0 ||
    !std::isfinite(action_server_timeout_sec_) || action_server_timeout_sec_ <= 0.0 ||
    !std::isfinite(child_result_timeout_sec_) ||
    child_result_timeout_sec_ < minimum_child_result_timeout_sec ||
    !std::isfinite(default_waypoint_timeout_sec_) || default_waypoint_timeout_sec_ <= 0.0)
  {
    throw std::invalid_argument("mission-execution timeouts must be finite and positive");
  }

  takeoff_client_ = rclcpp_action::create_client<swarm_interfaces::action::Takeoff>(
    this, "vehicle_control/takeoff");
  navigation_client_ = rclcpp_action::create_client<swarm_interfaces::action::NavigateTo>(
    this, "vehicle_control/navigate_to");
  orbit_client_ = rclcpp_action::create_client<swarm_interfaces::action::OrbitPoint>(
    this, "vehicle_control/orbit_point");
  land_client_ = rclcpp_action::create_client<swarm_interfaces::action::Land>(this, "vehicle_control/land");
  trajectory_client_ = rclcpp_action::create_client<swarm_interfaces::action::FollowTrajectory>(
    this, "vehicle_control/follow_trajectory");
  takeoff_server_ = create_operation_server<swarm_interfaces::action::Takeoff>(
    "mission_execution/takeoff", takeoff_client_);
  navigation_server_ = create_operation_server<swarm_interfaces::action::NavigateTo>(
    "mission_execution/navigate_to", navigation_client_);
  trajectory_server_ = create_operation_server<swarm_interfaces::action::FollowTrajectory>(
    "mission_execution/follow_trajectory", trajectory_client_);
  orbit_server_ = create_operation_server<swarm_interfaces::action::OrbitPoint>(
    "mission_execution/orbit_point", orbit_client_);
  land_server_ = create_operation_server<swarm_interfaces::action::Land>(
    "mission_execution/land", land_client_, true);
  state_subscription_ = create_subscription<swarm_interfaces::msg::VehicleState>(
    "vehicle_state", 10,
    [this](const swarm_interfaces::msg::VehicleState & message) {
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_vehicle_state_ = message;
        has_vehicle_state_ = true;
        vehicle_state_received_ = std::chrono::steady_clock::now();
      }
      state_condition_.notify_all();
    });

  mission_server_ = rclcpp_action::create_server<Mission>(
    this, "execute_waypoint_mission",
    [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Mission::Goal> goal) {
      bool is_valid = goal->waypoint_offsets.size() < std::numeric_limits<std::uint32_t>::max() &&
        std::isfinite(goal->takeoff_altitude_m) && goal->takeoff_altitude_m > 0.0F &&
        std::isfinite(goal->position_tolerance_m) && goal->position_tolerance_m >= 0.0F &&
        std::isfinite(goal->waypoint_timeout_sec) && goal->waypoint_timeout_sec >= 0.0F;
      if (goal->look_at_point_of_interest) {
        is_valid = is_valid && !goal->waypoint_offsets.empty() &&
          std::isfinite(goal->look_at_offset.x) && std::isfinite(goal->look_at_offset.y) &&
          std::isfinite(goal->look_at_offset.z);
      }
      if (goal->orbit_after_waypoints) {
        is_valid = is_valid && std::isfinite(goal->orbit_center_offset.x) &&
          std::isfinite(goal->orbit_center_offset.y) &&
          std::isfinite(goal->orbit_center_offset.z) &&
          std::isfinite(goal->orbit_radius_m) && goal->orbit_radius_m >= 1.0F &&
          std::isfinite(goal->orbit_tangential_speed_mps) &&
          goal->orbit_tangential_speed_mps > 0.0F &&
          std::isfinite(goal->orbit_revolutions) && goal->orbit_revolutions > 0.0F &&
          std::isfinite(goal->orbit_timeout_sec) && goal->orbit_timeout_sec > 0.0F;
      }
      for (const geometry_msgs::msg::Point & waypoint : goal->waypoint_offsets) {
        is_valid = is_valid && std::isfinite(waypoint.x) && std::isfinite(waypoint.y) &&
          std::isfinite(waypoint.z);
      }
      return reserve_command(is_valid) ?
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE : rclcpp_action::GoalResponse::REJECT;
    },
    [](const std::shared_ptr<MissionHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
    [this](const std::shared_ptr<MissionHandle> handle) {
      enqueue([this, handle]() {execute_mission<Mission>(handle, handle->get_goal());});
    });

  planner_client_ = create_client<PlanFlight>("mission_planner/plan_flight");
  flight_plan_server_ = rclcpp_action::create_server<FlightPlan>(
    this, "mission_execution/execute_flight_plan",
    [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const FlightPlan::Goal> goal) {
      return reserve_command(!goal->plan_json.empty() && goal->plan_json.size() <=
        maximum_plan_json_bytes) ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
        rclcpp_action::GoalResponse::REJECT;
    },
    [](const std::shared_ptr<FlightPlanHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
    [this](const std::shared_ptr<FlightPlanHandle> handle) {
      enqueue([this, handle]() {execute_flight_plan(handle);});
    });
  worker_thread_ = std::thread(&MissionExecutionNode::worker_loop, this);
}

MissionExecutionNode::~MissionExecutionNode()
{
  stopping_.store(true);
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    worker_stopping_ = true;
  }
  worker_condition_.notify_all();
  state_condition_.notify_all();
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

bool MissionExecutionNode::reserve_command(const bool is_goal_valid, const bool is_landing)
{
  if (!is_goal_valid) {
    RCLCPP_WARN(get_logger(), "Rejecting command: goal fields are invalid");
    return false;
  }
  std::lock_guard<std::mutex> lock(command_mutex_);
  if (stopping_.load() || landing_pending_ || (command_active_ && !is_landing)) {
    RCLCPP_WARN(get_logger(), "Rejecting command: another command or landing is active");
    return false;
  }
  command_active_ = true;
  landing_pending_ = is_landing;
  return true;
}

bool MissionExecutionNode::command_is_interrupted()
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  return landing_pending_ || stopping_.load() || !rclcpp::ok();
}

std::string MissionExecutionNode::state_error(
  const bool is_takeoff, const bool is_landing, const bool requires_yaw)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!has_vehicle_state_ || !latest_vehicle_state_.position_valid ||
    latest_vehicle_state_.header.frame_id != "local_enu" ||
    !is_finite_point(latest_vehicle_state_.position) ||
    std::chrono::steady_clock::now() - vehicle_state_received_ >
    std::chrono::duration<double>(telemetry_timeout_sec_))
  {
    return "No fresh, valid local_enu vehicle position was received";
  }
  if (requires_yaw && (!latest_vehicle_state_.yaw_valid ||
    !std::isfinite(latest_vehicle_state_.yaw_rad)))
  {
    return "Requested heading control requires valid vehicle yaw";
  }
  if ((is_takeoff && (latest_vehicle_state_.armed || !latest_vehicle_state_.landed)) ||
    (!is_takeoff && !is_landing && (!latest_vehicle_state_.armed ||
    latest_vehicle_state_.landed || !latest_vehicle_state_.offboard)))
  {
    return "Vehicle state is incompatible with the requested operation";
  }
  return {};
}

template<typename ActionT>
typename rclcpp_action::Server<ActionT>::SharedPtr MissionExecutionNode::create_operation_server(
  const std::string & name,
  const typename rclcpp_action::Client<ActionT>::SharedPtr & client,
  const bool is_landing)
{
  using Handle = rclcpp_action::ServerGoalHandle<ActionT>;
  return rclcpp_action::create_server<ActionT>(
    this, name,
    [this, is_landing](const rclcpp_action::GoalUUID &,
    std::shared_ptr<const typename ActionT::Goal> goal) {
      return reserve_command(is_valid_goal(*goal), is_landing) ?
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE : rclcpp_action::GoalResponse::REJECT;
    },
    [is_landing](const std::shared_ptr<Handle>) {
      // An accepted public landing remains committed through PX4 touchdown.
      return is_landing ? rclcpp_action::CancelResponse::REJECT :
             rclcpp_action::CancelResponse::ACCEPT;
    },
    [this, client, is_landing](const std::shared_ptr<Handle> handle) {
      enqueue([this, handle, client, is_landing]() {
        execute_operation<ActionT>(handle, client, is_landing);
      }, is_landing);
    });
}

template<typename ActionT>
void MissionExecutionNode::execute_operation(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> & goal_handle,
  const typename rclcpp_action::Client<ActionT>::SharedPtr & client, const bool is_landing)
{
  std::shared_ptr<typename ActionT::Result> result = std::make_shared<typename ActionT::Result>();
  if constexpr (std::is_same_v<ActionT, swarm_interfaces::action::NavigateTo> ||
    std::is_same_v<ActionT, swarm_interfaces::action::FollowTrajectory>)
  {
    result->final_distance_m = std::numeric_limits<float>::quiet_NaN();
    result->final_yaw_error_rad = std::numeric_limits<float>::quiet_NaN();
  }
  const std::shared_ptr<const typename ActionT::Goal> goal = goal_handle->get_goal();
  try {
    const std::function<bool()> is_interrupted = [this, goal_handle, is_landing]() {
        return goal_handle->is_canceling() || stopping_.load() || !rclcpp::ok() ||
               (!is_landing && command_is_interrupted());
      };
    bool requires_yaw = false;
    if constexpr (std::is_same_v<ActionT, swarm_interfaces::action::NavigateTo>) {
      requires_yaw = goal->look_at_target;
    } else if constexpr (std::is_same_v<ActionT, swarm_interfaces::action::FollowTrajectory>) {
      requires_yaw = goal->control_yaw;
    } else if constexpr (std::is_same_v<ActionT, swarm_interfaces::action::OrbitPoint>) {
      requires_yaw = goal->face_center;
    }
    const std::string state_failure = state_error(
      std::is_same_v<ActionT, swarm_interfaces::action::Takeoff>, is_landing, requires_yaw);
    if (!state_failure.empty()) {
      throw std::runtime_error(state_failure);
    }
    double result_timeout_sec = child_result_timeout_sec_;
    if constexpr (!std::is_same_v<ActionT, swarm_interfaces::action::Takeoff>) {
      result_timeout_sec = std::max(
        result_timeout_sec, static_cast<double>(goal->timeout_sec) + result_timeout_margin_sec);
    }
    const ChildActionOutcome outcome = run_child_action<ActionT>(
      client, child_pending_, *goal,
      deadline_after(action_server_timeout_sec_), deadline_after(result_timeout_sec),
      is_interrupted,
      [this, is_landing, requires_yaw]() {
        return state_error(
          std::is_same_v<ActionT, swarm_interfaces::action::Takeoff>, is_landing, requires_yaw);
      },
      [goal_handle](const typename ActionT::Feedback & feedback) {
        if (goal_handle->is_active()) {
          goal_handle->publish_feedback(std::make_shared<typename ActionT::Feedback>(feedback));
        }
      },
      [result](const typename ActionT::Result & child_result) {*result = child_result;});
    result->success = outcome.succeeded && !is_interrupted();
    result->message = !is_landing && command_is_interrupted() ?
      "Command superseded by explicit landing: " + outcome.message : outcome.message;
    if (result->success) {
      goal_handle->succeed(result);
    } else {
      finish_unsuccessful<ActionT>(goal_handle, result, outcome.cancellation_completed);
    }
  } catch (const std::exception & exception) {
    result->message = exception.what();
    finish_unsuccessful<ActionT>(goal_handle, result, !child_pending_->load());
  }
}

void MissionExecutionNode::enqueue(std::function<void()> job, const bool is_landing)
{
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    worker_jobs_.push_back([this, job = std::move(job), is_landing]() {
      try {
        job();
      } catch (const std::exception & exception) {
        RCLCPP_ERROR(get_logger(), "Unhandled command error: %s", exception.what());
      }
      std::lock_guard<std::mutex> command_lock(command_mutex_);
      if (is_landing) {
        landing_pending_ = false;
      }
      command_active_ = landing_pending_;
    });
  }
  worker_condition_.notify_one();
}

void MissionExecutionNode::worker_loop()
{
  for (;;) {
    std::function<void()> job;
    {
      std::unique_lock<std::mutex> lock(worker_mutex_);
      worker_condition_.wait(lock, [this]() {return worker_stopping_ || !worker_jobs_.empty();});
      if (worker_stopping_ && worker_jobs_.empty()) {
        return;
      }
      job = std::move(worker_jobs_.front());
      worker_jobs_.pop_front();
    }
    job();
  }
}

swarm_interfaces::msg::VehicleState MissionExecutionNode::starting_state(
  const std::function<bool()> & is_interrupted)
{
  const std::chrono::steady_clock::time_point deadline =
    deadline_after(starting_position_timeout_sec_);
  std::unique_lock<std::mutex> lock(state_mutex_);
  while (std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
    if (is_interrupted()) {
      throw std::runtime_error("Command interrupted before a starting position was available");
    }
    if (has_vehicle_state_ && latest_vehicle_state_.position_valid &&
      latest_vehicle_state_.header.frame_id == "local_enu" &&
      is_finite_point(latest_vehicle_state_.position) &&
      std::chrono::steady_clock::now() - vehicle_state_received_ <=
      std::chrono::duration<double>(telemetry_timeout_sec_))
    {
      return latest_vehicle_state_;
    }
    state_condition_.wait_for(lock, callback_poll_interval);
  }
  throw std::runtime_error("No fresh, valid local_enu vehicle position was received");
}

std::string MissionExecutionNode::frame_error(
  const swarm_interfaces::msg::VehicleState & captured_state)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!has_vehicle_state_ || !latest_vehicle_state_.position_valid ||
    latest_vehicle_state_.header.frame_id != "local_enu" ||
    !is_finite_point(latest_vehicle_state_.position) ||
    std::chrono::steady_clock::now() - vehicle_state_received_ >
    std::chrono::duration<double>(telemetry_timeout_sec_))
  {
    return "Vehicle position became stale or invalid during the translated mission";
  }
  if (latest_vehicle_state_.local_frame_revision != captured_state.local_frame_revision ||
    (captured_state.global_reference_valid && !latest_vehicle_state_.global_reference_valid))
  {
    return "Local coordinate frame changed; translated mission targets are no longer valid";
  }
  return {};
}

void MissionExecutionNode::execute_flight_plan(
  const std::shared_ptr<FlightPlanHandle> & goal_handle)
{
  const std::function<bool()> is_interrupted = [this, goal_handle]() {
      return goal_handle->is_canceling() || command_is_interrupted();
    };
  try {
    const swarm_interfaces::msg::VehicleState initial_state = starting_state(is_interrupted);
    const std::string state_failure = state_error(true, false, false);
    if (!state_failure.empty()) {
      throw std::runtime_error(state_failure);
    }
    std::shared_ptr<FlightPlan::Feedback> feedback = std::make_shared<FlightPlan::Feedback>();
    feedback->status = "Compiling JSON flight intent for " + vehicle_id_;
    goal_handle->publish_feedback(feedback);
    const std::chrono::steady_clock::time_point deadline = deadline_after(action_server_timeout_sec_);
    while (!planner_client_->wait_for_service(callback_poll_interval)) {
      if (is_interrupted()) {
        throw std::runtime_error("JSON mission interrupted before planning");
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        throw std::runtime_error("Mission planner service is unavailable");
      }
    }
    if (is_interrupted()) {
      throw std::runtime_error("JSON mission interrupted before planning");
    }
    std::shared_ptr<PlanFlight::Request> request = std::make_shared<PlanFlight::Request>();
    request->plan_json = goal_handle->get_goal()->plan_json;
    request->vehicle_id = vehicle_id_;
    request->starting_state = initial_state;
    rclcpp::Client<PlanFlight>::FutureAndRequestId pending = planner_client_->async_send_request(request);
    while (pending.wait_for(callback_poll_interval) != std::future_status::ready) {
      if (is_interrupted() || std::chrono::steady_clock::now() >= deadline) {
        planner_client_->remove_pending_request(pending);
        throw std::runtime_error(is_interrupted() ? "JSON mission interrupted during planning" :
                "Mission planner response timed out");
      }
    }
    const PlanFlight::Response::SharedPtr planned = pending.get();
    if (!planned->success) {
      throw std::runtime_error("Flight plan rejected: " + planned->message);
    }
    if (is_interrupted()) {
      throw std::runtime_error("JSON mission interrupted after planning");
    }
    const std::string frame_failure = frame_error(initial_state);
    if (!frame_failure.empty()) {
      throw std::runtime_error(frame_failure);
    }
    const std::function<bool(const geometry_msgs::msg::Point &, double)> is_bounded =
      [this](const geometry_msgs::msg::Point & offset, const double radius_m) {
        return is_finite_point(offset) && offset.z > 0.0 &&
               offset.z <= maximum_plan_altitude_m_ &&
               std::hypot(offset.x, offset.y) + radius_m <= maximum_plan_distance_m_;
      };
    bool is_valid = std::isfinite(planned->takeoff_altitude_m) &&
      planned->takeoff_altitude_m > 0.0F &&
      static_cast<double>(planned->takeoff_altitude_m) <= maximum_plan_altitude_m_ &&
      std::isfinite(planned->position_tolerance_m) && planned->position_tolerance_m > 0.0F &&
      planned->position_tolerance_m <= maximum_planned_position_tolerance_m &&
      planned->waypoint_offsets.size() <= maximum_planned_waypoints &&
      (!planned->waypoint_offsets.empty() || planned->orbit_after_waypoints);
    for (const geometry_msgs::msg::Point & offset : planned->waypoint_offsets) {
      is_valid = is_valid && is_bounded(offset, 0.0);
    }
    if (planned->orbit_after_waypoints) {
      is_valid = is_valid && std::isfinite(planned->orbit_radius_m) &&
        planned->orbit_radius_m >= minimum_orbit_radius_m &&
        is_bounded(planned->orbit_center_offset, static_cast<double>(planned->orbit_radius_m)) &&
        std::isfinite(planned->orbit_tangential_speed_mps) &&
        planned->orbit_tangential_speed_mps > 0.0F &&
        planned->orbit_tangential_speed_mps <= maximum_planned_orbit_speed_mps &&
        std::pow(static_cast<double>(planned->orbit_tangential_speed_mps), 2.0) /
        static_cast<double>(planned->orbit_radius_m) < maximum_planned_orbit_acceleration_mps2 &&
        planned->orbit_revolutions == 1.0F && std::isfinite(planned->orbit_timeout_sec) &&
        planned->orbit_timeout_sec > 0.0F && planned->orbit_timeout_sec <= maximum_planned_orbit_timeout_sec;
    }
    if (!is_valid) {
      throw std::runtime_error("Mission planner returned invalid or out-of-bounds instructions");
    }
    std::shared_ptr<Mission::Goal> mission = std::make_shared<Mission::Goal>();
    mission->takeoff_altitude_m = planned->takeoff_altitude_m;
    mission->waypoint_offsets = planned->waypoint_offsets;
    mission->position_tolerance_m = planned->position_tolerance_m;
    mission->land_after_waypoints = true;
    mission->orbit_after_waypoints = planned->orbit_after_waypoints;
    mission->orbit_center_offset = planned->orbit_center_offset;
    mission->orbit_radius_m = planned->orbit_radius_m;
    mission->orbit_tangential_speed_mps = planned->orbit_tangential_speed_mps;
    mission->orbit_revolutions = planned->orbit_revolutions;
    mission->orbit_timeout_sec = planned->orbit_timeout_sec;
    execute_mission<FlightPlan>(goal_handle, mission, initial_state);
  } catch (const std::exception & exception) {
    std::shared_ptr<FlightPlan::Result> result = std::make_shared<FlightPlan::Result>();
    result->message = exception.what();
    finish_unsuccessful<FlightPlan>(goal_handle, result);
  }
}

template<typename ActionT>
void MissionExecutionNode::execute_mission(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> & goal_handle,
  const std::shared_ptr<const Mission::Goal> & goal,
  const std::optional<swarm_interfaces::msg::VehicleState> & captured_state)
{
  std::shared_ptr<typename ActionT::Result> result = std::make_shared<typename ActionT::Result>();
  result->completed_waypoints = 0;
  result->orbit_completed = false;
  const std::function<bool()> parent_is_canceling = [this, goal_handle]() {
      return goal_handle->is_canceling() || command_is_interrupted();
    };
  std::shared_ptr<std::atomic<std::uint32_t>> completed_waypoints =
    std::make_shared<std::atomic<std::uint32_t>>(0);
  const std::function<void(std::uint32_t, float, const std::string &)> publish_feedback =
    [goal_handle, completed_waypoints](
      const std::uint32_t current_waypoint, const float distance_to_target,
      const std::string & status) {
      std::shared_ptr<typename ActionT::Feedback> feedback = std::make_shared<typename ActionT::Feedback>();
      feedback->current_waypoint = current_waypoint;
      feedback->completed_waypoints = completed_waypoints->load();
      feedback->distance_to_target_m = distance_to_target;
      feedback->status = status;
      if (goal_handle->is_active()) {
        goal_handle->publish_feedback(feedback);
      }
    };
  const std::function<void(float, const std::string &)> publish_orbit_feedback =
    [goal_handle, completed_waypoints](const float completed_revolutions,
      const std::string & status) {
      std::shared_ptr<typename ActionT::Feedback> feedback = std::make_shared<typename ActionT::Feedback>();
      feedback->current_waypoint = 0;
      feedback->completed_waypoints = completed_waypoints->load();
      feedback->distance_to_target_m = std::numeric_limits<float>::quiet_NaN();
      feedback->orbit_completed_revolutions = completed_revolutions;
      feedback->status = status;
      if (goal_handle->is_active()) {
        goal_handle->publish_feedback(feedback);
      }
    };

  RCLCPP_INFO(
    get_logger(), "Starting waypoint mission (waypoints=%zu, orbit=%s, land=%s)",
    goal->waypoint_offsets.size(), goal->orbit_after_waypoints ? "enabled" : "disabled",
    goal->land_after_waypoints ? "enabled" : "disabled");

  try {
    const swarm_interfaces::msg::VehicleState initial_state = captured_state ?
      *captured_state : starting_state(parent_is_canceling);
    const std::function<std::string()> execution_error = [this, captured_state]() {
        return captured_state ? frame_error(*captured_state) : std::string();
      };
    const geometry_msgs::msg::Point mission_origin = initial_state.position;

    const std::string takeoff_state_error = state_error(true, false, false);
    if (!takeoff_state_error.empty()) {
      throw std::runtime_error(takeoff_state_error);
    }
    publish_feedback(0, 0.0F, "Taking off");
    swarm_interfaces::action::Takeoff::Goal takeoff_goal;
    takeoff_goal.altitude_m = goal->takeoff_altitude_m;
    ChildActionOutcome takeoff_outcome = run_child_action<swarm_interfaces::action::Takeoff>(
      takeoff_client_, child_pending_, takeoff_goal,
      deadline_after(action_server_timeout_sec_), deadline_after(child_result_timeout_sec_),
      parent_is_canceling,
      [this]() {return state_error(true, false, false);},
      [publish_feedback](const swarm_interfaces::action::Takeoff::Feedback & feedback) {
        publish_feedback(
          0, std::abs(feedback.target_altitude_m - feedback.current_altitude_m), "Taking off");
      }, [](const swarm_interfaces::action::Takeoff::Result &) {}, execution_error);
    if (!takeoff_outcome.succeeded) {
      result->message = "Takeoff failed or was interrupted: " + takeoff_outcome.message;
      finish_unsuccessful<ActionT>(goal_handle, result, takeoff_outcome.cancellation_completed);
      return;
    }

    const double waypoint_timeout = goal->waypoint_timeout_sec == 0.0F ?
      default_waypoint_timeout_sec_ : static_cast<double>(goal->waypoint_timeout_sec);
    for (std::size_t index = 0; index < goal->waypoint_offsets.size(); ++index) {
      if (parent_is_canceling()) {
        result->message = "Mission canceled after " + std::to_string(result->completed_waypoints) +
          " completed waypoints";
        finish_unsuccessful<ActionT>(goal_handle, result);
        return;
      }
      const std::string navigation_state_error = state_error(
        false, false, goal->look_at_point_of_interest);
      if (!navigation_state_error.empty()) {
        throw std::runtime_error(navigation_state_error);
      }
      const geometry_msgs::msg::Point & offset = goal->waypoint_offsets[index];
      swarm_interfaces::action::NavigateTo::Goal navigation_goal;
      navigation_goal.target_position = add_waypoint_offset(mission_origin, offset);
      navigation_goal.frame_id = "local_enu";
      navigation_goal.look_at_target = goal->look_at_point_of_interest;
      if (goal->look_at_point_of_interest) {
        navigation_goal.look_at_position = add_waypoint_offset(
          mission_origin, goal->look_at_offset);
      }
      navigation_goal.position_tolerance_m = goal->position_tolerance_m;
      navigation_goal.timeout_sec = static_cast<float>(waypoint_timeout);
      const std::uint32_t current_waypoint = static_cast<std::uint32_t>(index + 1);
      publish_feedback(current_waypoint, 0.0F, "Navigating to waypoint");
      const double result_timeout = std::max(child_result_timeout_sec_, waypoint_timeout + 10.0);
      ChildActionOutcome navigation_outcome =
        run_child_action<swarm_interfaces::action::NavigateTo>(
        navigation_client_, child_pending_, navigation_goal,
        deadline_after(action_server_timeout_sec_), deadline_after(result_timeout),
        parent_is_canceling,
        [this, goal]() {return state_error(false, false, goal->look_at_point_of_interest);},
        [publish_feedback, current_waypoint](
          const swarm_interfaces::action::NavigateTo::Feedback & feedback) {
          publish_feedback(
            current_waypoint, feedback.distance_to_target_m, "Navigating to waypoint");
        }, [](const swarm_interfaces::action::NavigateTo::Result &) {}, execution_error);
      if (!navigation_outcome.succeeded) {
        result->message = "Waypoint " + std::to_string(current_waypoint) + " failed: " +
          navigation_outcome.message;
        finish_unsuccessful<ActionT>(goal_handle, result, navigation_outcome.cancellation_completed);
        return;
      }
      ++(*completed_waypoints);
      result->completed_waypoints = completed_waypoints->load();
    }

    if (goal->orbit_after_waypoints) {
      const std::string orbit_state_error = state_error(false, false, false);
      if (!orbit_state_error.empty()) {
        throw std::runtime_error(orbit_state_error);
      }
      RCLCPP_INFO(get_logger(), "Starting mission OrbitPoint child action");
      publish_feedback(0, 0.0F, "Orbiting point of interest");
      swarm_interfaces::action::OrbitPoint::Goal orbit_goal;
      orbit_goal.center_position = add_waypoint_offset(mission_origin, goal->orbit_center_offset);
      orbit_goal.frame_id = "local_enu";
      orbit_goal.radius_m = goal->orbit_radius_m;
      orbit_goal.clockwise = goal->orbit_clockwise;
      orbit_goal.tangential_speed_mps = goal->orbit_tangential_speed_mps;
      orbit_goal.revolutions = goal->orbit_revolutions;
      orbit_goal.timeout_sec = goal->orbit_timeout_sec;
      const double orbit_result_timeout = std::max(
        child_result_timeout_sec_, static_cast<double>(goal->orbit_timeout_sec) + 30.0);
      ChildActionOutcome orbit_outcome = run_child_action<swarm_interfaces::action::OrbitPoint>(
        orbit_client_, child_pending_, orbit_goal,
        deadline_after(action_server_timeout_sec_), deadline_after(orbit_result_timeout),
        parent_is_canceling,
        [this]() {return state_error(false, false, false);},
        [publish_orbit_feedback](const swarm_interfaces::action::OrbitPoint::Feedback & feedback) {
          publish_orbit_feedback(
            feedback.completed_revolutions, feedback.status);
        }, [](const swarm_interfaces::action::OrbitPoint::Result &) {}, execution_error);
      if (!orbit_outcome.succeeded) {
        result->message = "Orbit failed: " + orbit_outcome.message;
        RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
        finish_unsuccessful<ActionT>(goal_handle, result, orbit_outcome.cancellation_completed);
        return;
      }
      result->orbit_completed = true;
      RCLCPP_INFO(get_logger(), "Mission OrbitPoint child action completed");
    }

    if (goal->land_after_waypoints) {
      const std::string landing_state_error = state_error(false, true, false);
      if (!landing_state_error.empty()) {
        throw std::runtime_error(landing_state_error);
      }
      publish_feedback(0, 0.0F, "Landing");
      swarm_interfaces::action::Land::Goal land_goal;
      land_goal.timeout_sec = 0.0F;
      ChildActionOutcome land_outcome = run_child_action<swarm_interfaces::action::Land>(
        land_client_, child_pending_, land_goal,
        deadline_after(action_server_timeout_sec_), deadline_after(child_result_timeout_sec_),
        parent_is_canceling,
        [this]() {return state_error(false, true, false);},
        [publish_feedback](const swarm_interfaces::action::Land::Feedback & feedback) {
          publish_feedback(0, 0.0F, feedback.status);
        }, [](const swarm_interfaces::action::Land::Result &) {}, execution_error);
      if (!land_outcome.succeeded) {
        result->message = "Landing failed: " + land_outcome.message;
        finish_unsuccessful<ActionT>(goal_handle, result, land_outcome.cancellation_completed);
        return;
      }
    }

    if (parent_is_canceling()) {
      result->success = false;
      result->message = "Mission canceled before completion";
      finish_unsuccessful<ActionT>(goal_handle, result);
      return;
    }

    result->success = true;
    result->message = result->orbit_completed ?
      "Waypoint mission and orbit completed" : "Waypoint mission completed";
    RCLCPP_INFO(get_logger(), "%s", result->message.c_str());
    goal_handle->succeed(result);
  } catch (const std::exception & exception) {
    result->success = false;
    result->message = exception.what();
    RCLCPP_ERROR(get_logger(), "Waypoint mission failed: %s", exception.what());
    finish_unsuccessful<ActionT>(goal_handle, result, !child_pending_->load());
  }
}

}  // namespace mission_execution
