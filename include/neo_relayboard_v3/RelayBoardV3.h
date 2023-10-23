/*
 * RelayBoardV3.h
 *
 *  Created on: Jan 19, 2022
 *      Author: jaw
 */

#ifndef INCLUDE_neo_relayboard_v3_RelayBoardV3_H
#define INCLUDE_neo_relayboard_v3_RelayBoardV3_H

#include <neo_relayboard_v3/RelayBoardV3Base.hxx>
#include <pilot/PlatformInterfaceClient.hxx>
#include <pilot/SafetyInterfaceClient.hxx>
#include <pilot/ModuleLauncherClient.hxx>

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_srvs/srv/empty.hpp>
#include <neo_srvs2/srv/relay_board_set_relay.hpp>
#include <neo_srvs2/srv/io_board_set_dig_out.hpp>
#include <neo_srvs2/srv/relay_board_set_lcd_msg.hpp>
#include <neo_srvs2/srv/set_safety_field.hpp>
#include <neo_msgs2/msg/relay_board_v3.hpp>
#include <neo_msgs2/msg/safety_state.hpp>
#include <neo_msgs2/msg/kinematics_state.hpp>

namespace neo_relayboard_v3{


class RelayBoardV3 : public RelayBoardV3Base{
public:
	RelayBoardV3(const std::string &_vnx_name, std::shared_ptr<rclcpp::Node> node_handle);

protected:
	void main() override;

	void handle(std::shared_ptr<const pilot::SystemState> value) override;
	void handle(std::shared_ptr<const pilot::SafetyState> value) override;
	void handle(std::shared_ptr<const pilot::EmergencyState> value) override;
	void handle(std::shared_ptr<const pilot::BatteryState> value) override;
	void handle(std::shared_ptr<const pilot::PowerState> value) override;
	void handle(std::shared_ptr<const pilot::kinematics::bicycle::DriveState> value) override;
	void handle(std::shared_ptr<const pilot::kinematics::differential::DriveState> value) override;
	void handle(std::shared_ptr<const pilot::kinematics::mecanum::DriveState> value) override;
	void handle(std::shared_ptr<const pilot::kinematics::omnidrive::DriveState> value) override;
	void handle(std::shared_ptr<const pilot::RelayBoardV3Data> value) override;
	void handle(std::shared_ptr<const pilot::IOBoardData> value) override;
	void handle(std::shared_ptr<const pilot::USBoardData> value) override;

	void handle_JointTrajectory(std::shared_ptr<const trajectory_msgs::msg::JointTrajectory> trajectory, vnx::TopicPtr pilot_topic);
	void handle_KinState(std::shared_ptr<const neo_msgs2::msg::KinematicsState> state, vnx::TopicPtr pilot_topic);

private:
	std::shared_ptr<rclcpp::Node> nh;
	std::map<std::string, std::shared_ptr<rclcpp::SubscriptionBase>> import_subscribers;
	std::map<std::string, std::shared_ptr<rclcpp::PublisherBase>> export_publishers;

	std::shared_ptr<pilot::PlatformInterfaceClient> platform_interface;
	std::shared_ptr<pilot::SafetyInterfaceClient> safety_interface;
	std::shared_ptr<pilot::ModuleLauncherClient> module_launcher;

	bool board_initialized = false;
	std::shared_ptr<const pilot::PowerState> m_power_state;

	template<class T>
	void bulk_subscribe(std::function<void(std::shared_ptr<const T>, vnx::TopicPtr)> func, const std::map<std::string, vnx::TopicPtr> &mapping, const rclcpp::QoS &qos);
	template<class T>
	void publish_to_ros(std::shared_ptr<T> sample, const std::string &ros_topic, const rclcpp::QoS &qos);
	template<class T>
	void publish_to_ros(std::shared_ptr<T> sample, vnx::TopicPtr pilot_topic, const rclcpp::QoS &qos);

	void init_board();

	rclcpp::Service<neo_srvs2::srv::RelayBoardSetRelay>::SharedPtr srv_set_relay;
	rclcpp::Service<neo_srvs2::srv::IOBoardSetDigOut>::SharedPtr srv_io_board_set_dig_out;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_start_charging;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_stop_charging;
	rclcpp::Service<neo_srvs2::srv::RelayBoardSetLCDMsg>::SharedPtr srv_set_LCD_message;
	rclcpp::Service<neo_srvs2::srv::SetSafetyField>::SharedPtr srv_set_safety_field;

	bool is_shutdown = false;

	bool service_set_relay(std::shared_ptr<neo_srvs2::srv::RelayBoardSetRelay::Request> req, std::shared_ptr<neo_srvs2::srv::RelayBoardSetRelay::Response> res);
	bool service_set_digital_output(std::shared_ptr<neo_srvs2::srv::IOBoardSetDigOut::Request> req, std::shared_ptr<neo_srvs2::srv::IOBoardSetDigOut::Response> res);
	bool service_start_charging(std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res);
	bool service_stop_charging(std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res);
	bool service_set_LCD_message(std::shared_ptr<neo_srvs2::srv::RelayBoardSetLCDMsg::Request> req, std::shared_ptr<neo_srvs2::srv::RelayBoardSetLCDMsg::Response> res);
	bool service_set_safety_field(std::shared_ptr<neo_srvs2::srv::SetSafetyField::Request> req, std::shared_ptr<neo_srvs2::srv::SetSafetyField::Response> res);
};


} // neo_relayboard_v3


#endif /* INCLUDE_pilot_relayboardv3_RelayBoardV3_H */

