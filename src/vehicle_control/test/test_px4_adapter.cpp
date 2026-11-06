// Copyright 2026 DroneSwarming maintainers.
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>

#include "vehicle_control/px4_adapter.hpp"

namespace vehicle_control
{

TEST(Px4Adapter, PublishesCompleteOrbitCommandParameters)
{
  rclcpp::init(0, nullptr);
  {
    const std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>(
      "px4_orbit_command_test");
    Px4Adapter adapter(*node, "/fmu", 7);
    std::vector<px4_msgs::msg::VehicleCommand> published_commands;
    const rclcpp::QoS command_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local();
    const rclcpp::Subscription<px4_msgs::msg::VehicleCommand>::SharedPtr command_subscription =
      node->create_subscription<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", command_qos,
      [&published_commands](const px4_msgs::msg::VehicleCommand & message) {
        published_commands.push_back(message);
      });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    const std::chrono::steady_clock::time_point connection_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (command_subscription->get_publisher_count() == 0U &&
      std::chrono::steady_clock::now() < connection_deadline)
    {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_EQ(1U, command_subscription->get_publisher_count());

    const GlobalPosition center{42.1, -71.2, 125.5};
    const std::uint64_t generation = adapter.publish_orbit_command(-6.0, 1.25F, 0, center, 9876);
    static_cast<void>(generation);
    const std::chrono::steady_clock::time_point message_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (published_commands.empty() && std::chrono::steady_clock::now() < message_deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    ASSERT_EQ(1U, published_commands.size());
    const px4_msgs::msg::VehicleCommand & command = published_commands.front();
    EXPECT_EQ(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_ORBIT, command.command);
    EXPECT_FLOAT_EQ(-6.0F, command.param1);
    EXPECT_FLOAT_EQ(1.25F, command.param2);
    EXPECT_FLOAT_EQ(0.0F, command.param3);
    EXPECT_TRUE(std::isnan(command.param4));
    EXPECT_DOUBLE_EQ(center.latitude_deg, command.param5);
    EXPECT_DOUBLE_EQ(center.longitude_deg, command.param6);
    EXPECT_FLOAT_EQ(static_cast<float>(center.altitude_amsl_m), command.param7);
    EXPECT_EQ(7U, command.target_system);
    executor.remove_node(node);
  }
  rclcpp::shutdown();
}

TEST(Px4Adapter, AcceptsAcknowledgementWithoutExternalFlagForMatchingClient)
{
  rclcpp::init(0, nullptr);
  {
    const std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("px4_adapter_test");
    Px4Adapter adapter(*node, "/fmu", 1);
    const rclcpp::QoS acknowledgement_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local();
    const rclcpp::Publisher<px4_msgs::msg::VehicleCommandAck>::SharedPtr acknowledgement_publisher =
      node->create_publisher<px4_msgs::msg::VehicleCommandAck>(
      "/fmu/out/vehicle_command_ack", acknowledgement_qos);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    const std::uint32_t command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
    const std::uint64_t generation = adapter.publish_vehicle_command(command, 1.0F, 6.0F, 1);
    const std::chrono::steady_clock::time_point connection_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (acknowledgement_publisher->get_subscription_count() == 0U &&
      std::chrono::steady_clock::now() < connection_deadline)
    {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_EQ(1U, acknowledgement_publisher->get_subscription_count());

    px4_msgs::msg::VehicleCommandAck acknowledgement{};
    acknowledgement.command = command;
    acknowledgement.result = px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED;
    acknowledgement.result_param1 = 3U;
    acknowledgement.result_param2 = -17;
    acknowledgement.target_system = 1U;
    acknowledgement.target_component = 1U;
    acknowledgement.from_external = false;
    acknowledgement_publisher->publish(acknowledgement);

    const std::chrono::steady_clock::time_point ignored_ack_deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
    while (std::chrono::steady_clock::now() < ignored_ack_deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_FALSE(adapter.acknowledgement_after(command, generation).has_value());

    acknowledgement.target_system = 255U;
    acknowledgement.target_component = 1U;
    acknowledgement_publisher->publish(acknowledgement);

    std::optional<CommandAcknowledgement> received_acknowledgement;
    const std::chrono::steady_clock::time_point acknowledgement_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!received_acknowledgement.has_value() &&
      std::chrono::steady_clock::now() < acknowledgement_deadline)
    {
      executor.spin_some();
      received_acknowledgement = adapter.acknowledgement_after(command, generation);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    ASSERT_TRUE(received_acknowledgement.has_value());
    EXPECT_EQ(command, received_acknowledgement->command);
    EXPECT_EQ(px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED,
      received_acknowledgement->result);
    EXPECT_EQ(3U, received_acknowledgement->result_parameter_1);
    EXPECT_EQ(-17, received_acknowledgement->result_parameter_2);
    executor.remove_node(node);
  }
  rclcpp::shutdown();
}

TEST(Px4Adapter, ConvertsOptionalEnuYawToNedSetpointYaw)
{
  rclcpp::init(0, nullptr);
  {
    const std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>(
      "px4_yaw_setpoint_test");
    Px4Adapter adapter(*node, "/fmu", 1);
    std::vector<px4_msgs::msg::TrajectorySetpoint> published_setpoints;
    const rclcpp::QoS setpoint_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local();
    const rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_subscription =
      node->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", setpoint_qos,
      [&published_setpoints](const px4_msgs::msg::TrajectorySetpoint & message) {
        published_setpoints.push_back(message);
      });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    const std::chrono::steady_clock::time_point connection_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (setpoint_subscription->get_publisher_count() == 0U &&
      std::chrono::steady_clock::now() < connection_deadline)
    {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_EQ(1U, setpoint_subscription->get_publisher_count());

    adapter.publish_position_setpoint({2.0, 3.0, 4.0}, 1000, 0.0);
    const std::chrono::steady_clock::time_point message_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (published_setpoints.empty() && std::chrono::steady_clock::now() < message_deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    ASSERT_EQ(1U, published_setpoints.size());
    EXPECT_FLOAT_EQ(3.0F, published_setpoints.front().position[0]);
    EXPECT_FLOAT_EQ(2.0F, published_setpoints.front().position[1]);
    EXPECT_FLOAT_EQ(-4.0F, published_setpoints.front().position[2]);
    EXPECT_NEAR(1.57079632679, published_setpoints.front().yaw, 1.0e-6);

    adapter.publish_position_setpoint({2.0, 3.0, 4.0}, 2000, std::nullopt);
    const std::chrono::steady_clock::time_point uncontrolled_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (published_setpoints.size() < 2U &&
      std::chrono::steady_clock::now() < uncontrolled_deadline)
    {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_EQ(2U, published_setpoints.size());
    EXPECT_TRUE(std::isnan(published_setpoints.back().yaw));
    executor.remove_node(node);
  }
  rclcpp::shutdown();
}

}  // namespace vehicle_control
