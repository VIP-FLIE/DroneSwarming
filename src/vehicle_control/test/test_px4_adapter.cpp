// Copyright 2026 DroneSwarming maintainers.
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <thread>

#include <gtest/gtest.h>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <rclcpp/rclcpp.hpp>

#include "vehicle_control/px4_adapter.hpp"

namespace vehicle_control
{

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

}  // namespace vehicle_control
