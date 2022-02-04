/*
 * RelayBoardV3.cpp
 *
 *  Created on: Jan 19, 2022
 *      Author: jaw
 */

#include <neo_relayboard_v3/RelayBoardV3.h>
#include <pilot/kinematics/differential/DriveCmd.hxx>
#include <pilot/kinematics/mecanum/DriveCmd.hxx>
#include <pilot/kinematics/omnidrive/DriveCmd.hxx>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Range.h>
#include <neo_msgs/EmergencyStopState.h>
#include <neo_msgs/IOBoard.h>
#include <neo_msgs/USBoardV2.h>

#include <cmath>


namespace neo_relayboard_v3{

using namespace pilot;

inline ros::Time pilot_to_ros_time(const int64_t& time_usec){
	ros::Time time;
	time.fromNSec(time_usec * 1000);
	return time;
}


RelayBoardV3::RelayBoardV3(const std::string &_vnx_name):
	RelayBoardV3Base(_vnx_name)
{
}


void RelayBoardV3::init(){
}


void RelayBoardV3::main(){
	for(const auto &entry : topics_board_to_ros){
		subscribe(entry.first, 100);
	}
	for(const auto &entry : topics_ros_to_board){
		const auto &type = entry.first;
		if(type == "trajectory_msgs/JointTrajectory"){
			auto func = &RelayBoardV3::handle_JointTrajectory;
			bulk_subscribe<trajectory_msgs::JointTrajectory>(std::bind(func, this, std::placeholders::_1, std::placeholders::_2), entry.second);
		}else{
			ROS_ERROR_STREAM("Unsupported ROS type: " << type);
		}
	}
	for(const auto &topic : topics_from_board){
		subscribe(topic, 100);
	}
	platform_interface = std::make_shared<PlatformInterfaceClient>(platform_interface_server);

	ros_services.push_back(nh.advertiseService("set_relay", &RelayBoardV3::service_set_relay, this));
	ros_services.push_back(nh.advertiseService("/ioboard/set_digital_output", &RelayBoardV3::service_set_digital_output, this));
	ros_services.push_back(nh.advertiseService("start_charging", &RelayBoardV3::service_start_charging, this));
	ros_services.push_back(nh.advertiseService("stop_charging", &RelayBoardV3::service_stop_charging, this));
	ros_services.push_back(nh.advertiseService("set_LCD_msg", &RelayBoardV3::service_set_LCD_message, this));

	Super::main();
	ros::shutdown();
}


void RelayBoardV3::handle(std::shared_ptr<const pilot::SystemState> value){
	// TODO
}


void RelayBoardV3::handle(std::shared_ptr<const pilot::EmergencyState> value){
	auto out = boost::make_shared<neo_msgs::EmergencyStopState>();
	out->header.stamp = pilot_to_ros_time(value->time);

	out->emergency_button_stop = false;
	out->scanner_stop = false;
	if(value->code == safety_code_e::EMERGENCY_STOP){
		out->emergency_button_stop = true;
	}else if(value->code == safety_code_e::SCANNER_STOP){
		out->scanner_stop = true;
	}

	if(value->state == em_stop_state_e::STOPPED){
		out->emergency_state = neo_msgs::EmergencyStopState::EMSTOP;
	}else if(value->state == em_stop_state_e::CONFIRMED){
		out->emergency_state = neo_msgs::EmergencyStopState::EMCONFIRMED;
	}else{
		out->emergency_state = neo_msgs::EmergencyStopState::EMFREE;
	}

	publish_to_ros(out, vnx_sample->topic);
}


void RelayBoardV3::handle(std::shared_ptr<const pilot::BatteryState> value){
	auto out = boost::make_shared<sensor_msgs::BatteryState>();
	out->header.stamp = pilot_to_ros_time(value->time);

	out->voltage = value->voltage;
	out->current = value->current;
	out->charge = NAN;
	out->capacity = NAN;
	out->design_capacity = NAN;
	out->percentage = value->remaining;
	out->power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
	if(m_power_state){
		auto chrg = m_power_state->charging_state;
		if(chrg == charging_state_e::IS_CHARGING){
			out->power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
		}else if(chrg == charging_state_e::FINISHED){
			out->power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_FULL;
		}else{
			out->power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
	}
	out->power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
	out->power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
	if(value->type == battery_type_e::AGM){
		// lead
	}else if(value->type == battery_type_e::LFP){
		out->power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIFE;
	}
	out->present = true;

	publish_to_ros(out, vnx_sample->topic);
}


void RelayBoardV3::handle(std::shared_ptr<const pilot::PowerState> value){
	// to be merged with BatteryState
	m_power_state = value;
}


void RelayBoardV3::handle(std::shared_ptr<const pilot::kinematics::differential::DriveState> value){
	auto out = boost::make_shared<sensor_msgs::JointState>();
	out->header.stamp = pilot_to_ros_time(value->time);
	out->name.resize(2);
	out->position.resize(2);
	out->velocity.resize(2);
	out->name[0] = "wheel_front_left_joint";
	out->name[1] = "wheel_front_right_joint";
	out->position[0] = value->position.left;
	out->position[1] = value->position.right;
	out->velocity[0] = value->velocity.left;
	out->velocity[1] = value->velocity.right;
	if(value->has_torque) {
		out->effort.resize(2);
		out->effort[0] = value->torque.left;
		out->effort[1] = value->torque.right;
	}
	publish_to_ros(out, vnx_sample->topic);
}


void RelayBoardV3::handle(std::shared_ptr<const pilot::kinematics::mecanum::DriveState> value){
	auto out = boost::make_shared<sensor_msgs::JointState>();
	out->header.stamp = pilot_to_ros_time(value->time);
	out->name.resize(4);
	out->position.resize(4);
	out->velocity.resize(4);
	out->name[0] = "wheel_front_left_base_link";
	out->name[1] = "wheel_front_right_base_link";
	out->name[2] = "wheel_back_left_base_link";
	out->name[3] = "wheel_back_right_base_link";
	out->position[0] = value->position.front_left;
	out->position[1] = value->position.front_right;
	out->position[2] = value->position.back_left;
	out->position[3] = value->position.back_right;
	out->velocity[0] = value->velocity.front_left;
	out->velocity[1] = value->velocity.front_right;
	out->velocity[2] = value->velocity.back_left;
	out->velocity[3] = value->velocity.back_right;
	if(value->has_torque) {
		out->effort.resize(4);
		out->effort[0] = value->torque.front_left;
		out->effort[1] = value->torque.front_right;
		out->effort[2] = value->torque.back_left;
		out->effort[3] = value->torque.back_right;
	}
	publish_to_ros(out, vnx_sample->topic);
}


void RelayBoardV3::handle(std::shared_ptr<const pilot::kinematics::omnidrive::DriveState> value){
	auto out = boost::make_shared<sensor_msgs::JointState>();
	out->header.stamp = pilot_to_ros_time(value->time);
	out->name.resize(8);
	out->position.resize(8);
	out->velocity.resize(8);
	out->name[0] = "mpo_700_wheel_front_left_joint";
	out->name[1] = "mpo_700_caster_front_left_joint";
	out->name[2] = "mpo_700_wheel_back_left_joint";
	out->name[3] = "mpo_700_caster_back_left_joint";
	out->name[4] = "mpo_700_wheel_back_right_joint";
	out->name[5] = "mpo_700_caster_back_right_joint";
	out->name[6] = "mpo_700_wheel_front_right_joint";
	out->name[7] = "mpo_700_caster_front_right_joint";
	out->position[0] = value->drive_pos.front_left;
	out->position[1] = value->steer_pos.front_left;
	out->position[2] = value->drive_pos.back_left;
	out->position[3] = value->steer_pos.back_left;
	out->position[4] = value->drive_pos.back_right;
	out->position[5] = value->steer_pos.back_right;
	out->position[6] = value->drive_pos.front_right;
	out->position[7] = value->steer_pos.front_right;
	out->velocity[0] = value->drive_vel.front_left;
	out->velocity[1] = value->steer_vel.front_left;
	out->velocity[2] = value->drive_vel.back_left;
	out->velocity[3] = value->steer_vel.back_left;
	out->velocity[4] = value->drive_vel.back_right;
	out->velocity[5] = value->steer_vel.back_right;
	out->velocity[6] = value->drive_vel.front_right;
	out->velocity[7] = value->steer_vel.front_right;
	if(value->has_torque) {
		out->effort.resize(8);
		out->effort[0] = value->drive_torque.front_left;
		out->effort[1] = value->steer_torque.front_left;
		out->effort[2] = value->drive_torque.back_left;
		out->effort[3] = value->steer_torque.back_left;
		out->effort[4] = value->drive_torque.back_right;
		out->effort[5] = value->steer_torque.back_right;
		out->effort[6] = value->drive_torque.front_right;
		out->effort[7] = value->steer_torque.front_right;
	}
	publish_to_ros(out, vnx_sample->topic);
}


void RelayBoardV3::handle(std::shared_ptr<const pilot::RelayBoardData> value){
	// TODO
}


void RelayBoardV3::handle(std::shared_ptr<const pilot::IOBoardData> value){
	auto out = boost::make_shared<neo_msgs::IOBoard>();
	out->header.stamp = pilot_to_ros_time(value->time);

	for(size_t i=0; i<std::min(out->digital_inputs.size(), value->digital_input.size()); i++){
		out->digital_inputs[i] = value->digital_input[i];
	}
	for(size_t i=0; i<std::min(out->digital_outputs.size(), value->digital_output.size()); i++){
		out->digital_outputs[i] = value->digital_output[i];
	}
	for(size_t i=0; i<std::min(out->analog_inputs.size(), value->analog_input.size()); i++){
		out->analog_inputs[i] = value->analog_input[i];
	}

	publish_to_ros(out, vnx_sample->topic);
}


void RelayBoardV3::handle(std::shared_ptr<const pilot::USBoardData> value){
	{
		auto out = boost::make_shared<neo_msgs::USBoardV2>();
		out->header.stamp = pilot_to_ros_time(value->time);

		for(size_t i=0; i<std::min(out->sensor.size(), value->sensor.size()); i++){
			out->sensor[i] = value->sensor[i];
		}
		for(size_t i=0; i<std::min(out->analog.size(), value->analog_input.size()); i++){
			out->analog[i] = value->analog_input[i];
		}
		publish_to_ros(out, vnx_sample->topic);
	}

	for(size_t i=0; i<value->sensor.size(); i++){
		const std::string key = "us_sensor_" + std::to_string(i);
		auto find = topics_to_ros.find(key);
		if(find != topics_to_ros.end()){
			auto out = boost::make_shared<sensor_msgs::Range>();
			out->header.stamp = pilot_to_ros_time(value->time);
			out->header.frame_id = "us_" + std::to_string(i + 1) + "_link";
			out->radiation_type = sensor_msgs::Range::ULTRASOUND;
			out->field_of_view = 1.05;
			out->min_range = 0.1;
			out->max_range = 1.2;
			out->range = value->sensor[i];
			publish_to_ros(out, find->second);
		}
	}
}


void RelayBoardV3::handle_JointTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr &trajectory, vnx::TopicPtr pilot_topic){
	const trajectory_msgs::JointTrajectoryPoint &point = trajectory->points[0];

	if(kinematic_type == kinematic_type_e::DIFFERENTIAL){
		auto out = pilot::kinematics::differential::DriveCmd::create();
		out->time = vnx::get_time_micros();
		for(size_t i=0; i<std::min(trajectory->joint_names.size(), point.velocities.size()); i++){
			const auto &name = trajectory->joint_names[i];
			auto v = point.velocities[i];
			if(name == "wheel_front_left_joint"){
				out->velocity.left = v;
			}else if(name == "wheel_front_right_joint"){
				out->velocity.right = v;
			}else{
				throw std::logic_error("Unkwnown joint name: " + name);
			}
		}
		publish(out, pilot_topic);
	}else if(kinematic_type == kinematic_type_e::MECANUM){
		auto out = pilot::kinematics::mecanum::DriveCmd::create();
		out->time = vnx::get_time_micros();
		for(size_t i=0; i<std::min(trajectory->joint_names.size(), point.velocities.size()); i++){
			const auto &name = trajectory->joint_names[i];
			auto v = point.velocities[i];
			if(name == "wheel_front_left_base_link"){
				out->velocity.front_left = v;
			}else if(name == "wheel_front_right_base_link"){
				out->velocity.front_right = v;
			}else if(name == "wheel_back_left_base_link"){
				out->velocity.back_left = v;
			}else if(name == "wheel_back_right_base_link"){
				out->velocity.back_right = v;
			}else{
				throw std::logic_error("Unkwnown joint name: " + name);
			}
		}
		publish(out, pilot_topic);
	}else if(kinematic_type == kinematic_type_e::OMNIDRIVE){
		auto out = pilot::kinematics::omnidrive::DriveCmd::create();
		out->time = vnx::get_time_micros();
		for(size_t i=0; i<std::min(trajectory->joint_names.size(), std::min(point.positions.size(), point.velocities.size())); i++){
			const auto &name = trajectory->joint_names[i];
			auto p = point.positions[i];
			auto v = point.velocities[i];
			if(name == "mpo_700_wheel_front_left_joint"){
				out->drive_vel.front_left = v;
			}else if(name == "mpo_700_wheel_front_right_joint"){
				out->drive_vel.front_right = v;
			}else if(name == "mpo_700_wheel_back_left_joint"){
				out->drive_vel.back_left = v;
			}else if(name == "mpo_700_wheel_back_right_joint"){
				out->drive_vel.back_right = v;
			}else if(name == "mpo_700_caster_front_left_joint"){
				out->steer_pos.front_left = p;
			}else if(name == "mpo_700_caster_front_right_joint"){
				out->steer_pos.front_right = p;
			}else if(name == "mpo_700_caster_back_left_joint"){
				out->steer_pos.back_left = p;
			}else if(name == "mpo_700_caster_back_right_joint"){
				out->steer_pos.back_right = p;
			}else{
				throw std::logic_error("Unknown joint name: " + name);
			}
		}
		publish(out, pilot_topic);
	}else{
		throw std::logic_error("Unsupported kinematic type");
	}
}


bool RelayBoardV3::service_set_relay(neo_srvs::RelayBoardSetRelay::Request &req, neo_srvs::RelayBoardSetRelay::Response &res){
	try{
		platform_interface->set_relay(req.id, req.state);
		res.success = true;
	}catch(const std::exception &err){
		ROS_WARN_STREAM(err.what());
		res.success = false;
	}
	return true;
}


bool RelayBoardV3::service_set_digital_output(neo_srvs::IOBoardSetDigOut::Request &req, neo_srvs::IOBoardSetDigOut::Response &res){
	try{
		platform_interface->set_digital_output(req.id, req.state);
		res.success = true;
	}catch(const std::exception &err){
		ROS_WARN_STREAM(err.what());
		res.success = false;
	}
	return true;
}


bool RelayBoardV3::service_start_charging(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	try{
		platform_interface->start_charging();
		return true;
	}catch(const std::exception &err){
		ROS_WARN_STREAM(err.what());
		return false;
	}
}


bool RelayBoardV3::service_stop_charging(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	try{
		platform_interface->stop_charging();
		return true;
	}catch(const std::exception &err){
		ROS_WARN_STREAM(err.what());
		return false;
	}
}


bool RelayBoardV3::service_set_LCD_message(neo_srvs::RelayBoardSetLCDMsg::Request &req, neo_srvs::RelayBoardSetLCDMsg::Response &res){
	try{
		platform_interface->set_display_text(req.message);
		res.success = true;
	}catch(const std::exception &err){
		ROS_WARN_STREAM(err.what());
		res.success = false;
	}
	return true;
}


} // neo_relayboard_v3



