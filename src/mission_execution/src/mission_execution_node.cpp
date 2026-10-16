// Copyright 2026 DroneSwarming maintainers.
#include "mission_execution/mission_execution_node.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <future>
#include <optional>
#include <stdexcept>
#include <utility>

#include "mission_execution/geometry.hpp"

namespace mission_execution
{
namespace
{

constexpr std::chrono::milliseconds callback_poll_interval(100);
constexpr double minimum_child_result_timeout_sec = 60.0;

/// Stores synchronization state for one asynchronous child action.
template<typename ActionT>
struct ChildActionState
{
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;
  using WrappedResult = typename GoalHandle::WrappedResult;

  std::mutex mutex;
  std::condition_variable condition;
  std::optional<WrappedResult> result;
};

/// Reports success, cancellation, and diagnostic text from a child action.
struct ChildActionOutcome
{
  bool succeeded{false};
  bool canceled{false};
  std::string message;
};

/// Runs a child action and forwards its feedback.
/// Propagates parent cancellation and waits for the child's terminal result.
template<typename ActionT, typename GoalT, typename FeedbackCallback>
ChildActionOutcome run_child_action(
  const typename rclcpp_action::Client<ActionT>::SharedPtr & client,
  const GoalT & goal,
  const std::chrono::steady_clock::time_point & response_deadline,
  const std::chrono::steady_clock::time_point & result_deadline,
  const std::function<bool()> & parent_is_canceling,
  FeedbackCallback feedback_callback)
{
  using Client = rclcpp_action::Client<ActionT>;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;
  using WrappedResult = typename GoalHandle::WrappedResult;
  using Feedback = typename ActionT::Feedback;

  while (std::chrono::steady_clock::now() < response_deadline && rclcpp::ok()) {
    if (parent_is_canceling()) {
      return {false, true, "Mission canceled before starting the child action"};
    }
    if (client->wait_for_action_server(callback_poll_interval)) {
      break;
    }
  }
  if (!client->action_server_is_ready()) {
    return {false, false, "Vehicle-control action server is unavailable"};
  }

  std::shared_ptr<ChildActionState<ActionT>> child_state =
    std::make_shared<ChildActionState<ActionT>>();
  typename Client::SendGoalOptions options;
  options.feedback_callback = [feedback_callback](
      typename GoalHandle::SharedPtr,
      const std::shared_ptr<const Feedback> feedback) {
        feedback_callback(*feedback);
      };
  options.result_callback = [child_state](const WrappedResult & result) {
      {
        std::lock_guard<std::mutex> lock(child_state->mutex);
        child_state->result = result;
      }
      child_state->condition.notify_all();
    };
  std::shared_future<typename GoalHandle::SharedPtr> goal_response =
    client->async_send_goal(goal, options);

  while (goal_response.wait_for(callback_poll_interval) != std::future_status::ready) {
    if (std::chrono::steady_clock::now() >= response_deadline || !rclcpp::ok()) {
      return {false, false, "Timed out waiting for the child action goal response"};
    }
  }
  typename GoalHandle::SharedPtr child_goal = goal_response.get();
  if (!child_goal) {
    return {false, false, "Vehicle-control rejected the child action goal"};
  }

  bool cancellation_sent = false;
  bool child_timed_out = false;
  std::chrono::steady_clock::time_point cancel_deadline = result_deadline;
  for (;;) {
    if (parent_is_canceling() && !cancellation_sent) {
      client->async_cancel_goal(child_goal);
      cancellation_sent = true;
      cancel_deadline = result_deadline;
    }
    if (!cancellation_sent && std::chrono::steady_clock::now() >= result_deadline) {
      client->async_cancel_goal(child_goal);
      cancellation_sent = true;
      child_timed_out = true;
      cancel_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    }

    {
      std::unique_lock<std::mutex> lock(child_state->mutex);
      if (child_state->result) {
        const WrappedResult result = *child_state->result;
        if (parent_is_canceling() || result.code == rclcpp_action::ResultCode::CANCELED) {
          return {false, true, "Mission canceled; active child action finished"};
        }
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
          result.result->success)
        {
          return {true, false, result.result->message};
        }
        const std::string result_message = result.result ? result.result->message :
          "Child action returned no result";
        return {false, false, result_message};
      }
      child_state->condition.wait_for(lock, callback_poll_interval);
    }

    if (cancellation_sent && std::chrono::steady_clock::now() >= cancel_deadline) {
      return {false, false, child_timed_out ?
        "Child action timed out and did not finish after cancellation" :
        "Mission cancellation did not finish the active child action"};
    }
    if (!rclcpp::ok()) {
      return {false, true, "ROS shutdown interrupted the mission"};
    }
  }
}

/// Builds a steady-clock deadline duration_sec into the future.
std::chrono::steady_clock::time_point deadline_after(const double duration_sec)
{
  return std::chrono::steady_clock::now() +
         std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(duration_sec));
}

}  // namespace

MissionExecutionNode::MissionExecutionNode()
: Node("mission_execution")
{
  telemetry_timeout_sec_ = declare_parameter<double>("telemetry_timeout_sec", 1.0);
  starting_position_timeout_sec_ = declare_parameter<double>("starting_position_timeout_sec", 10.0);
  action_server_timeout_sec_ = declare_parameter<double>("action_server_timeout_sec", 10.0);
  child_result_timeout_sec_ = declare_parameter<double>("child_result_timeout_sec", 90.0);
  default_waypoint_timeout_sec_ = declare_parameter<double>("default_waypoint_timeout_sec", 30.0);
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
    this, "takeoff");
  navigation_client_ = rclcpp_action::create_client<swarm_interfaces::action::NavigateTo>(
    this, "navigate_to");
  land_client_ = rclcpp_action::create_client<swarm_interfaces::action::Land>(this, "land");
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
      bool is_valid = std::isfinite(goal->takeoff_altitude_m) && goal->takeoff_altitude_m > 0.0F &&
        std::isfinite(goal->position_tolerance_m) && goal->position_tolerance_m >= 0.0F &&
        std::isfinite(goal->waypoint_timeout_sec) && goal->waypoint_timeout_sec >= 0.0F;
      for (const geometry_msgs::msg::Point & waypoint : goal->waypoint_offsets) {
        is_valid = is_valid && std::isfinite(waypoint.x) && std::isfinite(waypoint.y) &&
          std::isfinite(waypoint.z);
      }
      return reserve_mission(is_valid) ?
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE : rclcpp_action::GoalResponse::REJECT;
    },
    [](const std::shared_ptr<MissionHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
    [this](const std::shared_ptr<MissionHandle> handle) {
      enqueue([this, handle]() {execute_mission(handle);});
    });

  worker_thread_ = std::thread(&MissionExecutionNode::worker_loop, this);
}

MissionExecutionNode::~MissionExecutionNode()
{
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

bool MissionExecutionNode::reserve_mission(const bool is_goal_valid)
{
  if (!is_goal_valid) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mission_mutex_);
  if (mission_active_) {
    return false;
  }
  mission_active_ = true;
  return true;
}

void MissionExecutionNode::enqueue(std::function<void()> job)
{
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    worker_jobs_.push_back(std::move(job));
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
    try {
      job();
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(get_logger(), "Unhandled mission error: %s", exception.what());
      std::lock_guard<std::mutex> lock(mission_mutex_);
      mission_active_ = false;
    }
  }
}

void MissionExecutionNode::execute_mission(const std::shared_ptr<MissionHandle> & goal_handle)
{
  std::shared_ptr<Mission::Result> result = std::make_shared<Mission::Result>();
  result->completed_waypoints = 0;
  const std::shared_ptr<const Mission::Goal> goal = goal_handle->get_goal();
  const std::function<bool()> parent_is_canceling = [goal_handle]() {
      return goal_handle->is_canceling() || !rclcpp::ok();
    };
  std::shared_ptr<std::atomic<std::uint32_t>> completed_waypoints =
    std::make_shared<std::atomic<std::uint32_t>>(0);
  const std::function<void(std::uint32_t, float, const std::string &)> publish_feedback =
    [goal_handle, completed_waypoints](
      const std::uint32_t current_waypoint, const float distance_to_target,
      const std::string & status) {
      std::shared_ptr<Mission::Feedback> feedback = std::make_shared<Mission::Feedback>();
      feedback->current_waypoint = current_waypoint;
      feedback->completed_waypoints = completed_waypoints->load();
      feedback->distance_to_target_m = distance_to_target;
      feedback->status = status;
      goal_handle->publish_feedback(feedback);
    };

  try {
    swarm_interfaces::msg::VehicleState starting_state;
    const std::chrono::steady_clock::time_point start_deadline =
      deadline_after(starting_position_timeout_sec_);
    {
      std::unique_lock<std::mutex> lock(state_mutex_);
      while (std::chrono::steady_clock::now() < start_deadline && rclcpp::ok()) {
        if (parent_is_canceling()) {
          result->message = "Mission canceled before a valid starting position was available";
          goal_handle->canceled(result);
          std::lock_guard<std::mutex> mission_lock(mission_mutex_);
          mission_active_ = false;
          return;
        }
        const std::chrono::steady_clock::duration age =
          std::chrono::steady_clock::now() - vehicle_state_received_;
        if (has_vehicle_state_ && latest_vehicle_state_.position_valid &&
          latest_vehicle_state_.header.frame_id == "local_enu" &&
          age <= std::chrono::duration<double>(telemetry_timeout_sec_))
        {
          starting_state = latest_vehicle_state_;
          break;
        }
        state_condition_.wait_for(lock, callback_poll_interval);
      }
    }
    if (starting_state.header.frame_id != "local_enu" || !starting_state.position_valid) {
      throw std::runtime_error("No fresh, valid local_enu vehicle position was received");
    }
    const geometry_msgs::msg::Point mission_origin = starting_state.position;

    publish_feedback(0, 0.0F, "Taking off");
    swarm_interfaces::action::Takeoff::Goal takeoff_goal;
    takeoff_goal.altitude_m = goal->takeoff_altitude_m;
    ChildActionOutcome takeoff_outcome = run_child_action<swarm_interfaces::action::Takeoff>(
      takeoff_client_, takeoff_goal,
      deadline_after(action_server_timeout_sec_), deadline_after(child_result_timeout_sec_),
      parent_is_canceling,
      [publish_feedback](const swarm_interfaces::action::Takeoff::Feedback & feedback) {
        publish_feedback(
          0, std::abs(feedback.target_altitude_m - feedback.current_altitude_m), "Taking off");
      });
    if (!takeoff_outcome.succeeded) {
      result->message = "Takeoff failed: " + takeoff_outcome.message;
      if (takeoff_outcome.canceled || parent_is_canceling()) {
        goal_handle->canceled(result);
      } else {
        goal_handle->abort(result);
      }
      std::lock_guard<std::mutex> lock(mission_mutex_);
      mission_active_ = false;
      return;
    }

    const double waypoint_timeout = goal->waypoint_timeout_sec == 0.0F ?
      default_waypoint_timeout_sec_ : goal->waypoint_timeout_sec;
    for (std::size_t index = 0; index < goal->waypoint_offsets.size(); ++index) {
      if (parent_is_canceling()) {
        result->message = "Mission canceled after " + std::to_string(result->completed_waypoints) +
          " completed waypoints";
        goal_handle->canceled(result);
        std::lock_guard<std::mutex> lock(mission_mutex_);
        mission_active_ = false;
        return;
      }
      const geometry_msgs::msg::Point & offset = goal->waypoint_offsets[index];
      swarm_interfaces::action::NavigateTo::Goal navigation_goal;
      navigation_goal.target_position = add_waypoint_offset(mission_origin, offset);
      navigation_goal.frame_id = "local_enu";
      navigation_goal.position_tolerance_m = goal->position_tolerance_m;
      navigation_goal.timeout_sec = static_cast<float>(waypoint_timeout);
      const std::uint32_t current_waypoint = static_cast<std::uint32_t>(index + 1);
      publish_feedback(current_waypoint, 0.0F, "Navigating to waypoint");
      const double result_timeout = std::max(child_result_timeout_sec_, waypoint_timeout + 10.0);
      ChildActionOutcome navigation_outcome =
        run_child_action<swarm_interfaces::action::NavigateTo>(
        navigation_client_, navigation_goal,
        deadline_after(action_server_timeout_sec_), deadline_after(result_timeout),
        parent_is_canceling,
        [publish_feedback, current_waypoint](
          const swarm_interfaces::action::NavigateTo::Feedback & feedback) {
          publish_feedback(
            current_waypoint, feedback.distance_to_target_m, "Navigating to waypoint");
        });
      if (!navigation_outcome.succeeded) {
        result->message = "Waypoint " + std::to_string(current_waypoint) + " failed: " +
          navigation_outcome.message;
        if (navigation_outcome.canceled || parent_is_canceling()) {
          goal_handle->canceled(result);
        } else {
          goal_handle->abort(result);
        }
        std::lock_guard<std::mutex> lock(mission_mutex_);
        mission_active_ = false;
        return;
      }
      ++(*completed_waypoints);
      result->completed_waypoints = completed_waypoints->load();
    }

    if (goal->land_after_waypoints) {
      publish_feedback(0, 0.0F, "Landing");
      swarm_interfaces::action::Land::Goal land_goal;
      land_goal.timeout_sec = 0.0F;
      ChildActionOutcome land_outcome = run_child_action<swarm_interfaces::action::Land>(
        land_client_, land_goal,
        deadline_after(action_server_timeout_sec_), deadline_after(child_result_timeout_sec_),
        parent_is_canceling,
        [publish_feedback](const swarm_interfaces::action::Land::Feedback & feedback) {
          publish_feedback(0, 0.0F, feedback.status);
        });
      if (!land_outcome.succeeded) {
        result->message = "Landing failed: " + land_outcome.message;
        if (land_outcome.canceled || parent_is_canceling()) {
          goal_handle->canceled(result);
        } else {
          goal_handle->abort(result);
        }
        std::lock_guard<std::mutex> lock(mission_mutex_);
        mission_active_ = false;
        return;
      }
    }

    result->success = true;
    result->message = "Waypoint mission completed";
    goal_handle->succeed(result);
  } catch (const std::exception & exception) {
    result->success = false;
    result->message = exception.what();
    if (parent_is_canceling()) {
      goal_handle->canceled(result);
    } else {
      goal_handle->abort(result);
    }
  }
  {
    std::lock_guard<std::mutex> lock(mission_mutex_);
    mission_active_ = false;
  }
}

}  // namespace mission_execution
