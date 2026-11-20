// Copyright 2026 DroneSwarming maintainers.
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "mission_execution/mission_execution_node.hpp"

namespace mission_execution
{
namespace
{

using Takeoff = swarm_interfaces::action::Takeoff;
using NavigateTo = swarm_interfaces::action::NavigateTo;
using FollowTrajectory = swarm_interfaces::action::FollowTrajectory;
using OrbitPoint = swarm_interfaces::action::OrbitPoint;
using Land = swarm_interfaces::action::Land;
using Mission = swarm_interfaces::action::ExecuteWaypointMission;
constexpr std::chrono::seconds test_timeout(3);

/// Polls observable ROS state without blocking executor callbacks.
bool wait_until(const std::function<bool()> & predicate)
{
  const std::chrono::steady_clock::time_point deadline =
    std::chrono::steady_clock::now() + test_timeout;
  while (!predicate() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return predicate();
}

/// Leaves each child running until the test supplies its terminal result.
template<typename ActionT>
class ControlledAction
{
public:
  using Handle = rclcpp_action::ServerGoalHandle<ActionT>;

  ControlledAction(const std::shared_ptr<rclcpp::Node> & node, const std::string & name)
  {
    server = rclcpp_action::create_server<ActionT>(
      node, "vehicle_control/" + name,
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const typename ActionT::Goal>) {
        std::this_thread::sleep_for(response_delay);
        return should_reject ? rclcpp_action::GoalResponse::REJECT :
               rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<Handle>) {
        ++cancellations;
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<Handle> handle) {
        std::lock_guard<std::mutex> lock(mutex_);
        handles_.push_back(handle);
      });
  }

  std::shared_ptr<Handle> goal(const std::size_t index = 0)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return index < handles_.size() ? handles_[index] : nullptr;
  }

  std::size_t count()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return handles_.size();
  }

  void succeed(const std::size_t index = 0)
  {
    std::shared_ptr<typename ActionT::Result> result = std::make_shared<typename ActionT::Result>();
    result->success = true;
    result->message = "Child completed";
    goal(index)->succeed(result);
  }

  void finish_canceled(const std::size_t index = 0)
  {
    goal(index)->canceled(std::make_shared<typename ActionT::Result>());
  }

  typename rclcpp_action::Server<ActionT>::SharedPtr server;
  std::atomic<unsigned int> cancellations{0};
  bool should_reject{false};
  std::chrono::milliseconds response_delay{0};

private:
  std::mutex mutex_;
  std::vector<std::shared_ptr<Handle>> handles_;
};

class CommandAuthority : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    static unsigned int test_number = 0;
    const std::string test_namespace = "/command_test_" + std::to_string(++test_number);
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__ns:=" + test_namespace});
    options.parameter_overrides({
      rclcpp::Parameter("telemetry_timeout_sec", 0.3),
      rclcpp::Parameter("starting_position_timeout_sec", 0.5),
      rclcpp::Parameter("action_server_timeout_sec", 0.5)});
    mission_ = std::make_shared<MissionExecutionNode>(options);
    peer_ = std::make_shared<rclcpp::Node>("command_test_peer", test_namespace);
    takeoff_ = std::make_unique<ControlledAction<Takeoff>>(peer_, "takeoff");
    navigation_ = std::make_unique<ControlledAction<NavigateTo>>(peer_, "navigate_to");
    trajectory_ = std::make_unique<ControlledAction<FollowTrajectory>>(peer_, "follow_trajectory");
    orbit_ = std::make_unique<ControlledAction<OrbitPoint>>(peer_, "orbit_point");
    land_ = std::make_unique<ControlledAction<Land>>(peer_, "land");
    publisher_ = peer_->create_publisher<swarm_interfaces::msg::VehicleState>("vehicle_state", 10);
    state_.header.frame_id = "local_enu";
    state_.position_valid = true;
    state_.yaw_valid = true;
    state_.landed = true;
    timer_ = peer_->create_wall_timer(std::chrono::milliseconds(20), [this]() {
      std::lock_guard<std::mutex> lock(state_mutex_);
      publisher_->publish(state_);
    });
    executor_.add_node(mission_);
    executor_.add_node(peer_);
    spin_thread_ = std::thread([this]() {executor_.spin();});
    ASSERT_TRUE(wait_until([this]() {return publisher_->get_subscription_count() == 1U;}));
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
  }

  void TearDown() override
  {
    executor_.cancel();
    spin_thread_.join();
    executor_.remove_node(mission_);
    executor_.remove_node(peer_);
    mission_.reset();
  }

  void set_airborne()
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      state_.armed = true;
      state_.landed = false;
      state_.offboard = true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
  }

  template<typename ActionT>
  typename rclcpp_action::Client<ActionT>::SharedPtr client(const std::string & operation)
  {
    typename rclcpp_action::Client<ActionT>::SharedPtr action_client =
      rclcpp_action::create_client<ActionT>(peer_, operation == "execute_waypoint_mission" ?
      operation : "mission_execution/" + operation);
    EXPECT_TRUE(action_client->wait_for_action_server(test_timeout));
    return action_client;
  }

  template<typename ActionT>
  typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr send(
    const typename rclcpp_action::Client<ActionT>::SharedPtr & action_client,
    const typename ActionT::Goal & goal)
  {
    std::shared_future<typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr> response =
      action_client->async_send_goal(goal);
    EXPECT_EQ(response.wait_for(test_timeout), std::future_status::ready);
    return response.get();
  }

  NavigateTo::Goal navigation_goal()
  {
    NavigateTo::Goal goal;
    goal.frame_id = "local_enu";
    goal.target_position.x = 5.0;
    goal.timeout_sec = 20.0F;
    return goal;
  }

  OrbitPoint::Goal orbit_goal()
  {
    OrbitPoint::Goal goal;
    goal.frame_id = "local_enu";
    goal.radius_m = 3.0F;
    goal.tangential_speed_mps = 1.0F;
    goal.revolutions = 1.0F;
    goal.timeout_sec = 30.0F;
    return goal;
  }

  Mission::Goal mission_goal()
  {
    Mission::Goal goal;
    goal.takeoff_altitude_m = 3.0F;
    geometry_msgs::msg::Point offset;
    offset.x = 3.0;
    offset.z = 3.0;
    goal.waypoint_offsets.push_back(offset);
    offset.y = 3.0;
    goal.waypoint_offsets.push_back(offset);
    return goal;
  }

  std::shared_ptr<MissionExecutionNode> mission_;
  std::shared_ptr<rclcpp::Node> peer_;
  rclcpp::executors::MultiThreadedExecutor executor_{rclcpp::ExecutorOptions(), 3};
  std::thread spin_thread_;
  std::unique_ptr<ControlledAction<Takeoff>> takeoff_;
  std::unique_ptr<ControlledAction<NavigateTo>> navigation_;
  std::unique_ptr<ControlledAction<FollowTrajectory>> trajectory_;
  std::unique_ptr<ControlledAction<OrbitPoint>> orbit_;
  std::unique_ptr<ControlledAction<Land>> land_;
  rclcpp::Publisher<swarm_interfaces::msg::VehicleState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex state_mutex_;
  swarm_interfaces::msg::VehicleState state_;
};

TEST_F(CommandAuthority, TakeoffAndLandingUseInternalActions)
{
  const rclcpp_action::Client<Takeoff>::SharedPtr takeoff_client = client<Takeoff>("takeoff");
  Takeoff::Goal goal;
  goal.altitude_m = 3.0F;
  const rclcpp_action::ClientGoalHandle<Takeoff>::SharedPtr handle = send<Takeoff>(takeoff_client, goal);
  ASSERT_NE(handle, nullptr);
  ASSERT_TRUE(wait_until([this]() {return takeoff_->count() == 1U;}));
  EXPECT_FLOAT_EQ(takeoff_->goal()->get_goal()->altitude_m, 3.0F);
  takeoff_->succeed();
  EXPECT_EQ(takeoff_client->async_get_result(handle).get().code, rclcpp_action::ResultCode::SUCCEEDED);
  const rclcpp_action::Client<Land>::SharedPtr land_client = client<Land>("land");
  const rclcpp_action::ClientGoalHandle<Land>::SharedPtr landing = send<Land>(land_client, Land::Goal());
  ASSERT_NE(landing, nullptr);
  ASSERT_TRUE(wait_until([this]() {return land_->count() == 1U;}));
  EXPECT_TRUE(land_client->async_cancel_goal(landing).get()->goals_canceling.empty());
  std::shared_ptr<Land::Result> landed = std::make_shared<Land::Result>();
  landed->success = true;
  landed->landed = true;
  land_->goal()->succeed(landed);
  EXPECT_TRUE(land_client->async_get_result(landing).get().result->landed);
}

TEST_F(CommandAuthority, TrajectoryPreservesFeedbackAndFailureMeasurements)
{
  set_airborne();
  const rclcpp_action::Client<FollowTrajectory>::SharedPtr action_client =
    client<FollowTrajectory>("follow_trajectory");
  FollowTrajectory::Goal goal;
  goal.points.resize(2);
  goal.points[1].time_from_start_sec = 2.0F;
  goal.points[1].position.x = 1.0;
  std::atomic<bool> received_feedback{false};
  rclcpp_action::Client<FollowTrajectory>::SendGoalOptions options;
  options.feedback_callback = [&received_feedback](
    rclcpp_action::ClientGoalHandle<FollowTrajectory>::SharedPtr,
    const std::shared_ptr<const FollowTrajectory::Feedback> feedback) {
      received_feedback.store(feedback->elapsed_sec == 1.0F && feedback->distance_to_target_m == 0.25F);
    };
  const rclcpp_action::ClientGoalHandle<FollowTrajectory>::SharedPtr handle =
    action_client->async_send_goal(goal, options).get();
  ASSERT_NE(handle, nullptr);
  ASSERT_TRUE(wait_until([this]() {return trajectory_->count() == 1U;}));
  EXPECT_EQ(trajectory_->goal()->get_goal()->points.size(), 2U);
  std::shared_ptr<FollowTrajectory::Feedback> feedback = std::make_shared<FollowTrajectory::Feedback>();
  feedback->elapsed_sec = 1.0F;
  feedback->distance_to_target_m = 0.25F;
  trajectory_->goal()->publish_feedback(feedback);
  ASSERT_TRUE(wait_until([&received_feedback]() {return received_feedback.load();}));
  std::shared_ptr<FollowTrajectory::Result> result = std::make_shared<FollowTrajectory::Result>();
  result->message = "Tracking failed";
  result->final_distance_m = 2.5F;
  result->final_yaw_error_rad = 0.4F;
  trajectory_->goal()->abort(result);
  const rclcpp_action::ClientGoalHandle<FollowTrajectory>::WrappedResult outcome =
    action_client->async_get_result(handle).get();
  EXPECT_EQ(outcome.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_FALSE(outcome.result->success);
  EXPECT_EQ(outcome.result->message, "Tracking failed");
  EXPECT_FLOAT_EQ(outcome.result->final_distance_m, 2.5F);
  EXPECT_FLOAT_EQ(outcome.result->final_yaw_error_rad, 0.4F);
}

TEST_F(CommandAuthority, NavigationCancellationWaitsForChildAndRejectsConflicts)
{
  set_airborne();
  const rclcpp_action::Client<NavigateTo>::SharedPtr action_client = client<NavigateTo>("navigate_to");
  const rclcpp_action::ClientGoalHandle<NavigateTo>::SharedPtr handle =
    send<NavigateTo>(action_client, navigation_goal());
  ASSERT_NE(handle, nullptr);
  ASSERT_TRUE(wait_until([this]() {return navigation_->count() == 1U;}));
  const rclcpp_action::Client<Mission>::SharedPtr mission_client = client<Mission>("execute_waypoint_mission");
  EXPECT_EQ(send<Mission>(mission_client, mission_goal()), nullptr);
  EXPECT_EQ(send<NavigateTo>(action_client, navigation_goal()), nullptr);
  EXPECT_FALSE(action_client->async_cancel_goal(handle).get()->goals_canceling.empty());
  ASSERT_TRUE(wait_until([this]() {return navigation_->cancellations.load() == 1U;}));
  std::shared_future<rclcpp_action::ClientGoalHandle<NavigateTo>::WrappedResult> outcome =
    action_client->async_get_result(handle);
  EXPECT_EQ(outcome.wait_for(std::chrono::milliseconds(150)), std::future_status::timeout);
  navigation_->finish_canceled();
  ASSERT_EQ(outcome.wait_for(test_timeout), std::future_status::ready);
  EXPECT_EQ(outcome.get().code, rclcpp_action::ResultCode::CANCELED);
}

TEST_F(CommandAuthority, OrbitCancellationPreservesProgress)
{
  set_airborne();
  const rclcpp_action::Client<OrbitPoint>::SharedPtr action_client = client<OrbitPoint>("orbit_point");
  const rclcpp_action::ClientGoalHandle<OrbitPoint>::SharedPtr handle =
    send<OrbitPoint>(action_client, orbit_goal());
  ASSERT_NE(handle, nullptr);
  ASSERT_TRUE(wait_until([this]() {return orbit_->count() == 1U;}));
  action_client->async_cancel_goal(handle).get();
  ASSERT_TRUE(wait_until([this]() {return orbit_->cancellations.load() == 1U;}));
  std::shared_ptr<OrbitPoint::Result> result = std::make_shared<OrbitPoint::Result>();
  result->completed_revolutions = 0.25F;
  orbit_->goal()->canceled(result);
  const rclcpp_action::ClientGoalHandle<OrbitPoint>::WrappedResult outcome =
    action_client->async_get_result(handle).get();
  EXPECT_EQ(outcome.code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_FLOAT_EQ(outcome.result->completed_revolutions, 0.25F);
}

TEST_F(CommandAuthority, ExplicitLandingInterruptsNavigationAfterChildTerminates)
{
  set_airborne();
  const rclcpp_action::Client<NavigateTo>::SharedPtr action_client = client<NavigateTo>("navigate_to");
  const rclcpp_action::ClientGoalHandle<NavigateTo>::SharedPtr handle =
    send<NavigateTo>(action_client, navigation_goal());
  ASSERT_TRUE(wait_until([this]() {return navigation_->count() == 1U;}));
  const rclcpp_action::Client<Land>::SharedPtr land_client = client<Land>("land");
  Land::Goal invalid_landing;
  invalid_landing.timeout_sec = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(send<Land>(land_client, invalid_landing), nullptr);
  EXPECT_EQ(navigation_->cancellations.load(), 0U);
  const rclcpp_action::ClientGoalHandle<Land>::SharedPtr landing = send<Land>(land_client, Land::Goal());
  ASSERT_NE(landing, nullptr);
  ASSERT_TRUE(wait_until([this]() {return navigation_->cancellations.load() == 1U;}));
  EXPECT_EQ(land_->count(), 0U);
  navigation_->finish_canceled();
  ASSERT_TRUE(wait_until([this]() {return land_->count() == 1U;}));
  EXPECT_EQ(action_client->async_get_result(handle).get().code, rclcpp_action::ResultCode::ABORTED);
  land_->succeed();
  EXPECT_TRUE(land_client->async_get_result(landing).get().result->success);
}

TEST_F(CommandAuthority, StaleAndNonfiniteStateCannotDispatch)
{
  const rclcpp_action::Client<Takeoff>::SharedPtr action_client = client<Takeoff>("takeoff");
  timer_->cancel();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  Takeoff::Goal goal;
  goal.altitude_m = 3.0F;
  const rclcpp_action::ClientGoalHandle<Takeoff>::SharedPtr stale = send<Takeoff>(action_client, goal);
  ASSERT_NE(stale, nullptr);
  EXPECT_EQ(action_client->async_get_result(stale).get().code, rclcpp_action::ResultCode::ABORTED);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_.position.x = std::numeric_limits<double>::infinity();
  }
  timer_->reset();
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  const rclcpp_action::ClientGoalHandle<Takeoff>::SharedPtr invalid = send<Takeoff>(action_client, goal);
  ASSERT_NE(invalid, nullptr);
  EXPECT_EQ(action_client->async_get_result(invalid).get().code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(takeoff_->count(), 0U);
  const rclcpp_action::Client<NavigateTo>::SharedPtr navigation_client = client<NavigateTo>("navigate_to");
  const rclcpp_action::ClientGoalHandle<NavigateTo>::SharedPtr navigation =
    send<NavigateTo>(navigation_client, navigation_goal());
  ASSERT_NE(navigation, nullptr);
  const rclcpp_action::ClientGoalHandle<NavigateTo>::WrappedResult navigation_result =
    navigation_client->async_get_result(navigation).get();
  EXPECT_EQ(navigation_result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_TRUE(std::isnan(navigation_result.result->final_distance_m));
  EXPECT_TRUE(std::isnan(navigation_result.result->final_yaw_error_rad));
}

TEST_F(CommandAuthority, UnavailableAndRejectedChildServersReturnFailure)
{
  takeoff_->server.reset();
  const rclcpp_action::Client<Takeoff>::SharedPtr action_client = client<Takeoff>("takeoff");
  Takeoff::Goal goal;
  goal.altitude_m = 3.0F;
  const rclcpp_action::ClientGoalHandle<Takeoff>::SharedPtr handle = send<Takeoff>(action_client, goal);
  ASSERT_NE(handle, nullptr);
  const rclcpp_action::ClientGoalHandle<Takeoff>::WrappedResult outcome =
    action_client->async_get_result(handle).get();
  EXPECT_EQ(outcome.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_NE(outcome.result->message.find("unavailable"), std::string::npos);
  set_airborne();
  orbit_->should_reject = true;
  const rclcpp_action::Client<OrbitPoint>::SharedPtr orbit_client = client<OrbitPoint>("orbit_point");
  const rclcpp_action::ClientGoalHandle<OrbitPoint>::SharedPtr rejected =
    send<OrbitPoint>(orbit_client, orbit_goal());
  ASSERT_NE(rejected, nullptr);
  EXPECT_EQ(orbit_client->async_get_result(rejected).get().code, rclcpp_action::ResultCode::ABORTED);
}

TEST_F(CommandAuthority, MissionOffsetsUseOneOriginAndCompleteOptionalLanding)
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_.position.x = 10.0;
    state_.position.y = 20.0;
    state_.position.z = 1.0;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  const rclcpp_action::Client<Mission>::SharedPtr action_client = client<Mission>("execute_waypoint_mission");
  Mission::Goal goal = mission_goal();
  goal.land_after_waypoints = true;
  const rclcpp_action::ClientGoalHandle<Mission>::SharedPtr handle = send<Mission>(action_client, goal);
  ASSERT_NE(handle, nullptr);
  ASSERT_TRUE(wait_until([this]() {return takeoff_->count() == 1U;}));
  set_airborne();
  takeoff_->succeed();
  ASSERT_TRUE(wait_until([this]() {return navigation_->count() == 1U;}));
  EXPECT_DOUBLE_EQ(navigation_->goal()->get_goal()->target_position.x, 13.0);
  EXPECT_DOUBLE_EQ(navigation_->goal()->get_goal()->target_position.z, 4.0);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_.position.x = 13.0;
    state_.position.z = 4.0;
  }
  navigation_->succeed();
  ASSERT_TRUE(wait_until([this]() {return navigation_->count() == 2U;}));
  EXPECT_DOUBLE_EQ(navigation_->goal(1)->get_goal()->target_position.x, 13.0);
  EXPECT_DOUBLE_EQ(navigation_->goal(1)->get_goal()->target_position.y, 23.0);
  navigation_->succeed(1);
  ASSERT_TRUE(wait_until([this]() {return land_->count() == 1U;}));
  land_->succeed();
  const rclcpp_action::ClientGoalHandle<Mission>::WrappedResult outcome =
    action_client->async_get_result(handle).get();
  EXPECT_EQ(outcome.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(outcome.result->completed_waypoints, 2U);
}

TEST_F(CommandAuthority, ExplicitLandingPreemptsWaypointMission)
{
  const rclcpp_action::Client<Mission>::SharedPtr action_client = client<Mission>("execute_waypoint_mission");
  const rclcpp_action::ClientGoalHandle<Mission>::SharedPtr handle = send<Mission>(action_client, mission_goal());
  ASSERT_TRUE(wait_until([this]() {return takeoff_->count() == 1U;}));
  set_airborne();
  takeoff_->succeed();
  ASSERT_TRUE(wait_until([this]() {return navigation_->count() == 1U;}));
  const rclcpp_action::Client<Land>::SharedPtr land_client = client<Land>("land");
  const rclcpp_action::ClientGoalHandle<Land>::SharedPtr landing = send<Land>(land_client, Land::Goal());
  ASSERT_NE(landing, nullptr);
  ASSERT_TRUE(wait_until([this]() {return navigation_->cancellations.load() == 1U;}));
  navigation_->finish_canceled();
  ASSERT_TRUE(wait_until([this]() {return land_->count() == 1U;}));
  land_->succeed();
  const rclcpp_action::ClientGoalHandle<Mission>::WrappedResult outcome =
    action_client->async_get_result(handle).get();
  EXPECT_EQ(outcome.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(outcome.result->completed_waypoints, 0U);
  EXPECT_EQ(navigation_->count(), 1U);
  EXPECT_TRUE(land_client->async_get_result(landing).get().result->success);
}

TEST_F(CommandAuthority, LateAcceptanceIsCanceledAndBlocksOverlappingLanding)
{
  set_airborne();
  navigation_->response_delay = std::chrono::milliseconds(800);
  const rclcpp_action::Client<NavigateTo>::SharedPtr action_client = client<NavigateTo>("navigate_to");
  const rclcpp_action::ClientGoalHandle<NavigateTo>::SharedPtr handle =
    send<NavigateTo>(action_client, navigation_goal());
  ASSERT_NE(handle, nullptr);
  const rclcpp_action::ClientGoalHandle<NavigateTo>::WrappedResult outcome =
    action_client->async_get_result(handle).get();
  EXPECT_EQ(outcome.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_TRUE(wait_until([this]() {return navigation_->cancellations.load() == 1U;}));
  const rclcpp_action::Client<Land>::SharedPtr land_client = client<Land>("land");
  const rclcpp_action::ClientGoalHandle<Land>::SharedPtr blocked = send<Land>(land_client, Land::Goal());
  ASSERT_NE(blocked, nullptr);
  EXPECT_EQ(land_client->async_get_result(blocked).get().code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(land_->count(), 0U);
  navigation_->finish_canceled();
  const rclcpp_action::ClientGoalHandle<Land>::SharedPtr landing = send<Land>(land_client, Land::Goal());
  ASSERT_NE(landing, nullptr);
  ASSERT_TRUE(wait_until([this]() {return land_->count() == 1U;}));
  land_->succeed();
  EXPECT_TRUE(land_client->async_get_result(landing).get().result->success);
}

TEST_F(CommandAuthority, MissionCancellationWaitsForNavigationTerminalResult)
{
  const rclcpp_action::Client<Mission>::SharedPtr action_client = client<Mission>("execute_waypoint_mission");
  const rclcpp_action::ClientGoalHandle<Mission>::SharedPtr handle = send<Mission>(action_client, mission_goal());
  ASSERT_TRUE(wait_until([this]() {return takeoff_->count() == 1U;}));
  set_airborne();
  takeoff_->succeed();
  ASSERT_TRUE(wait_until([this]() {return navigation_->count() == 1U;}));
  action_client->async_cancel_goal(handle).get();
  ASSERT_TRUE(wait_until([this]() {return navigation_->cancellations.load() == 1U;}));
  std::shared_future<rclcpp_action::ClientGoalHandle<Mission>::WrappedResult> outcome =
    action_client->async_get_result(handle);
  EXPECT_EQ(outcome.wait_for(std::chrono::milliseconds(150)), std::future_status::timeout);
  navigation_->finish_canceled();
  ASSERT_EQ(outcome.wait_for(test_timeout), std::future_status::ready);
  EXPECT_EQ(outcome.get().code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(outcome.get().result->completed_waypoints, 0U);
  EXPECT_EQ(navigation_->count(), 1U);
  EXPECT_EQ(land_->count(), 0U);
}

TEST_F(CommandAuthority, MissionOrbitFailurePreservesWaypointCountAndSkipsLanding)
{
  const rclcpp_action::Client<Mission>::SharedPtr action_client = client<Mission>("execute_waypoint_mission");
  Mission::Goal goal = mission_goal();
  goal.waypoint_offsets.resize(1);
  goal.orbit_after_waypoints = true;
  goal.orbit_radius_m = 3.0F;
  goal.orbit_tangential_speed_mps = 1.0F;
  goal.orbit_revolutions = 1.0F;
  goal.orbit_timeout_sec = 30.0F;
  goal.land_after_waypoints = true;
  const rclcpp_action::ClientGoalHandle<Mission>::SharedPtr handle = send<Mission>(action_client, goal);
  ASSERT_TRUE(wait_until([this]() {return takeoff_->count() == 1U;}));
  set_airborne();
  takeoff_->succeed();
  ASSERT_TRUE(wait_until([this]() {return navigation_->count() == 1U;}));
  navigation_->succeed();
  ASSERT_TRUE(wait_until([this]() {return orbit_->count() == 1U;}));
  std::shared_ptr<OrbitPoint::Result> failure = std::make_shared<OrbitPoint::Result>();
  failure->message = "Orbit hold recovery failed";
  orbit_->goal()->abort(failure);
  const rclcpp_action::ClientGoalHandle<Mission>::WrappedResult outcome =
    action_client->async_get_result(handle).get();
  EXPECT_EQ(outcome.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(outcome.result->completed_waypoints, 1U);
  EXPECT_FALSE(outcome.result->orbit_completed);
  EXPECT_EQ(land_->count(), 0U);
}

TEST_F(CommandAuthority, MissionCannotAdvanceWithInvalidState)
{
  const rclcpp_action::Client<Mission>::SharedPtr action_client = client<Mission>("execute_waypoint_mission");
  const rclcpp_action::ClientGoalHandle<Mission>::SharedPtr handle = send<Mission>(action_client, mission_goal());
  ASSERT_TRUE(wait_until([this]() {return takeoff_->count() == 1U;}));
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_.position_valid = false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  takeoff_->succeed();
  const rclcpp_action::ClientGoalHandle<Mission>::WrappedResult outcome =
    action_client->async_get_result(handle).get();
  EXPECT_EQ(outcome.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(navigation_->count(), 0U);
}

TEST_F(CommandAuthority, UnconfirmedCancellationAbortsAndBlocksDispatch)
{
  set_airborne();
  const rclcpp_action::Client<NavigateTo>::SharedPtr action_client = client<NavigateTo>("navigate_to");
  const rclcpp_action::ClientGoalHandle<NavigateTo>::SharedPtr handle =
    send<NavigateTo>(action_client, navigation_goal());
  ASSERT_TRUE(wait_until([this]() {return navigation_->count() == 1U;}));
  action_client->async_cancel_goal(handle).get();
  ASSERT_TRUE(wait_until([this]() {return navigation_->cancellations.load() == 1U;}));
  std::shared_future<rclcpp_action::ClientGoalHandle<NavigateTo>::WrappedResult> outcome =
    action_client->async_get_result(handle);
  ASSERT_EQ(outcome.wait_for(std::chrono::seconds(18)), std::future_status::ready);
  EXPECT_EQ(outcome.get().code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_NE(outcome.get().result->message.find("no confirmed terminal result"), std::string::npos);
  const rclcpp_action::Client<Land>::SharedPtr land_client = client<Land>("land");
  const rclcpp_action::ClientGoalHandle<Land>::SharedPtr landing = send<Land>(land_client, Land::Goal());
  ASSERT_NE(landing, nullptr);
  EXPECT_EQ(land_client->async_get_result(landing).get().code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(land_->count(), 0U);
  navigation_->finish_canceled();
}

TEST_F(CommandAuthority, FailedOrbitRecoveryAfterCancellationReturnsFailure)
{
  set_airborne();
  const rclcpp_action::Client<OrbitPoint>::SharedPtr action_client = client<OrbitPoint>("orbit_point");
  const rclcpp_action::ClientGoalHandle<OrbitPoint>::SharedPtr handle =
    send<OrbitPoint>(action_client, orbit_goal());
  ASSERT_TRUE(wait_until([this]() {return orbit_->count() == 1U;}));
  action_client->async_cancel_goal(handle).get();
  ASSERT_TRUE(wait_until([this]() {return orbit_->cancellations.load() == 1U;}));
  std::shared_ptr<OrbitPoint::Result> failure = std::make_shared<OrbitPoint::Result>();
  failure->message = "Offboard hold recovery failed";
  orbit_->goal()->abort(failure);
  const rclcpp_action::ClientGoalHandle<OrbitPoint>::WrappedResult outcome =
    action_client->async_get_result(handle).get();
  EXPECT_EQ(outcome.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_FALSE(outcome.result->success);
  EXPECT_EQ(outcome.result->message, failure->message);
}

}  // namespace
}  // namespace mission_execution
