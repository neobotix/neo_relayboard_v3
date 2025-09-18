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
#include <pilot/kinematics/KinematicsState.hxx>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Range.h>
#include <neo_msgs/EmergencyStopState.h>
#include <neo_msgs/IOBoard.h>
#include <neo_msgs/USBoardV2.h>
#include <neo_msgs/RelayBoardV3.h>
#include <neo_msgs/SafetyState.h>
#include <neo_msgs/KinematicsState.h>
#include <neo_msgs/SafetyMode.h>

#include <std_msgs/ColorRGBA.h>

#include <cmath>
#include <vnx/vnx.h>
#include <unistd.h>
#include <ros/ros.h>

#include <memory>
#include <functional>
using namespace std::placeholders;

namespace neo_relayboard_v3{

using namespace pilot;

inline
ros::Time pilot_to_ros_time(const int64_t& time_usec)
{
	return ros::Time(time_usec * 1e-6);
}

RelayBoardV3::RelayBoardV3(const std::string &_vnx_name, ros::NodeHandle& node_handle):
	RelayBoardV3Base(_vnx_name)
{
	nh = &node_handle;
}

void RelayBoardV3::main(){
	for(const auto &entry : topics_board_to_ros){
		subscribe(entry.first, 100);
	}
	for (const auto& entry : topics_ros_to_board) {
		const auto& ros_type = entry.first; 
		const auto& ros_topic = entry.second;
		for (const auto& topic : ros_topic) {
			const auto& ros_topic = topic.first;
			const auto& pilot_topic = topic.second;
			ros::Subscriber subs;
			if (ros_type == "trajectory_msgs/JointTrajectory") {
				subs = nh->subscribe<trajectory_msgs::JointTrajectory>(
					ros_topic, 10,
					std::bind(&RelayBoardV3::handle_JointTrajectory, this, std::placeholders::_1, pilot_topic));
			} else if (ros_type == "neo_msgs/KinematicsState") {
				subs = nh->subscribe<neo_msgs::KinematicsState>(
					ros_topic, 10,
					std::bind(&RelayBoardV3::handle_KinematicsState, this,  std::placeholders::_1, pilot_topic));
			} else {
				log(WARN) << "Unsupported ROS type: " << ros_type;
			}
			import_subscribers[ros_topic] = subs;
		}
	}

	for(const auto &topic : topics_from_board){
		subscribe(topic, 100);
	}
	srv_set_relay = nh->advertiseService("set_relay", &RelayBoardV3::service_set_relay, this);
	srv_set_digital_output = nh->advertiseService("ioboard/set_digital_output", &RelayBoardV3::service_set_digital_output, this);
	srv_start_charging = nh->advertiseService("start_charging", &RelayBoardV3::service_start_charging, this);
	srv_stop_charging = nh->advertiseService("stop_charging", &RelayBoardV3::service_stop_charging, this);
	srv_shutdown_platform = nh->advertiseService("shutdown_platform", &RelayBoardV3::service_shutdown_platform, this);
	srv_set_safety_field = nh->advertiseService("set_safety_field", &RelayBoardV3::service_set_safety_field, this);
	srv_set_safety_mode = nh->advertiseService("set_safety_mode", &RelayBoardV3::service_set_safety_mode, this);
	srv_set_leds = nh->advertiseService("set_leds", &RelayBoardV3::service_set_leds, this);
	platform_interface = std::make_shared<PlatformInterfaceClient>(platform_interface_server);
	safety_interface = std::make_shared<SafetyInterfaceClient>(safety_server);
	module_launcher = std::make_shared<ModuleLauncherClient>(launcher_server);

	

	if(board_init_interval_ms > 0){
		set_timer_millis(board_init_interval_ms, std::bind(&RelayBoardV3::init_board, this));
	}
	if(update_interval_ms > 0){
		set_timer_millis(update_interval_ms, std::bind(&RelayBoardV3::update, this));
	}

	Super::main();
	ros::shutdown();
}

void RelayBoardV3::handle(std::shared_ptr<const vnx::LogMsg> value){
	const std::string message = value->get_output();
	switch(value->level){
	case ERROR:
		ROS_ERROR("%s", message.c_str());
		break;
	case WARN:
		ROS_WARN("%s", message.c_str());
		break;
	case DEBUG:
		ROS_DEBUG("%s", message.c_str());
		break;
	case INFO:
	default:
		ROS_INFO("%s", message.c_str());
		break;
	}
}

void RelayBoardV3::handle(std::shared_ptr<const pilot::Incident> value){
	const auto key = value->event.get_key();
	auto iter = incident_map.find(key);
	if(iter != incident_map.end()) {
		if(value->is_cleared) {
			on_incident(value);
			incident_map.erase(iter);
		} else {
			iter->second = value;
		}
	}else{
		on_incident(value);
		if(value->is_active){
			incident_map[key] = value;
		}
	}
}

void RelayBoardV3::handle(std::shared_ptr<const pilot::SystemState> value){
	m_system_error.clear();
	for(const auto &code : value->system_errors){
		handle(Incident::create_ex(
			value->time,
			event_t::create_ex(event_type_e::ERROR, "SystemState", code.get_type_name(), code.to_string_value()),
			0, {}, true, 3000)
		);
		m_system_error.push_back(code.to_string_value());    
	}
	if (value->is_shutdown && !is_shutdown) {
		ROS_INFO("-----------SHUTDOWN Signal from RelayBoardV3----------");
		is_shutdown = true;
		ros::shutdown();
		vnx::trigger_shutdown();
		usleep(2000);
		system("sudo halt -p");
	}
}

void RelayBoardV3::handle(std::shared_ptr<const pilot::SafetyState> value){
	auto out = std::make_shared<neo_msgs::SafetyState>();
	out->time = value->time;
	out->current_safety_field = value->current_safety_field;
	out->triggered_cutoff_paths.resize(8, false);
	for (char path : value->triggered_cutoff_paths) {
		if (path >= 0 && path < 8) {
			out->triggered_cutoff_paths[path] = true;
		}
	}
	publish_to_ros(out, vnx_sample->topic, 10);
}

void RelayBoardV3::handle(std::shared_ptr<const pilot::EmergencyState> value){
	auto out = std::make_shared<neo_msgs::EmergencyStopState>();
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
	publish_to_ros(out, vnx_sample->topic, 10);
}

void RelayBoardV3::handle(std::shared_ptr<const pilot::BatteryState> value){
	auto out = std::make_shared<sensor_msgs::BatteryState>();
	out->header.stamp = pilot_to_ros_time(value->time);
	out->voltage = value->voltage;
	out->current = NAN;
	if(m_power_state && m_power_state->is_charging){
		out->current = m_power_state->charging_current;
	}else if(value->current){
		out->current = *value->current;
	}
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
	publish_to_ros(out, vnx_sample->topic, 10);
}

void RelayBoardV3::handle(std::shared_ptr<const pilot::PowerState> value){
	m_power_state = value;
}

void RelayBoardV3::handle(std::shared_ptr<const pilot::kinematics::bicycle::DriveState> value){
	const std::string dont_optimize_away_the_library = vnx::to_string(*value);
	// TODO
}

void RelayBoardV3::handle(std::shared_ptr<const pilot::kinematics::differential::DriveState> value){
	auto out = std::make_shared<sensor_msgs::JointState>();
	out->header.stamp = pilot_to_ros_time(value->time);
	out->name.resize(2);
	out->position.resize(2);
	out->velocity.resize(2);
	out->name[0] = "wheel_left_joint";
	out->name[1] = "wheel_right_joint";
	out->position[0] = value->position.left;
	out->position[1] = value->position.right;
	out->velocity[0] = value->velocity.left;
	out->velocity[1] = value->velocity.right;
	if(value->has_torque) {
		out->effort.resize(2);
		out->effort[0] = value->torque.left;
		out->effort[1] = value->torque.right;
	}
	publish_to_ros(out, vnx_sample->topic, 10);
}

void RelayBoardV3::handle(std::shared_ptr<const pilot::kinematics::mecanum::DriveState> value){
	auto out = std::make_shared<sensor_msgs::JointState>();
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
	publish_to_ros(out, vnx_sample->topic, 10);
}

void RelayBoardV3::handle(std::shared_ptr<const pilot::kinematics::omnidrive::DriveState> value){
	auto out = std::make_shared<sensor_msgs::JointState>();
	out->header.stamp = ros::Time::now();
	std::vector<std::string> joint_names;
	std::vector<double> joint_positions;
	std::vector<double> joint_velocities;
	std::vector<double> joint_efforts;
	std::map<pilot::kinematics::position_code_e, std::string> joint_translation = {
		{pilot::kinematics::position_code_e::FRONT_LEFT, "front_left_joint"},
		{pilot::kinematics::position_code_e::BACK_LEFT, "back_left_joint"},
		{pilot::kinematics::position_code_e::BACK_RIGHT, "back_right_joint"},
		{pilot::kinematics::position_code_e::FRONT_RIGHT, "front_right_joint"},
		{pilot::kinematics::position_code_e::FRONT, "front_joint"},
		{pilot::kinematics::position_code_e::BACK, "back_joint"}
	};
	for (const auto& pos : joint_translation) {
		if (value->steer_pos.positions.count(pos.first)) {
			const auto &suffix = pos.second;
			joint_names.push_back("wheel_" + suffix);
			joint_names.push_back("caster_" + suffix);
			joint_positions.push_back(value->drive_pos.positions.count(pos.first) ? value->drive_pos.get(pos.first) : 0.0);
			joint_velocities.push_back(value->drive_vel.get(pos.first));
			joint_positions.push_back(value->steer_pos.get(pos.first));
			joint_velocities.push_back(value->steer_vel.velocities.count(pos.first) ? value->steer_vel.get(pos.first) : 0.0);
			if (value->has_torque) {
				joint_efforts.push_back(value->drive_torque.get(pos.first));    
				joint_efforts.push_back(value->steer_torque.get(pos.first));    
			}
		}
	}
	size_t num_joints = joint_names.size();
	out->name.resize(num_joints);
	out->position.resize(num_joints);
	out->velocity.resize(num_joints);
	if (value->has_torque) {
		out->effort.resize(num_joints);
	}
	for (size_t i = 0; i < num_joints; i++) {
		out->name[i] = joint_names[i];
		out->position[i] = joint_positions[i];
		out->velocity[i] = joint_velocities[i];
		if (value->has_torque) {
			out->effort[i] = joint_efforts[i];
		}
	}
	publish_to_ros(out, vnx_sample->topic, 10);
}

void RelayBoardV3::handle(std::shared_ptr<const pilot::RelayBoardV3Data> value){
	auto out = std::make_shared<neo_msgs::RelayBoardV3>();
	out->time = value->time;
	out->firmware_version = value->firmware_version;
	out->uptime = value->uptime;
	out->ambient_temperature = value->ambient_temperature;
	// Copy std::array fields from std::vector<bool> or std::vector<uint8_t>
	for (size_t i = 0; i < out->relay_states.size() && i < value->relay_states.size(); ++i) {
		out->relay_states[i] = value->relay_states[i];
	}
	for (size_t i = 0; i < out->digital_input_states.size() && i < value->digital_input_states.size(); ++i) {
		out->digital_input_states[i] = value->digital_input_states[i];
	}
	for (size_t i = 0; i < out->keypad_button_states.size() && i < value->keypad_button_states.size(); ++i) {
		out->keypad_button_states[i] = value->keypad_button_states[i];
	}
	out->key_switch_off_state = value->key_switch_off_state;
	out->release_structure_state = value->release_structure_state;
	for (const auto& pair: value->led_states) {
		if (pair.first == led_color_e::RED) {
			out->led_state.r = pair.second;
		} else if (pair.first == led_color_e::GREEN) {
			out->led_state.g = pair.second;
		} else if (pair.first == led_color_e::BLUE) {
			out->led_state.b = pair.second;
		}
	}
	out->system_error = m_system_error;
	publish_to_ros(out, vnx_sample->topic, 10);
}

void RelayBoardV3::handle(std::shared_ptr<const pilot::IOBoardData> value){
	auto out = std::make_shared<neo_msgs::IOBoard>();
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
	publish_to_ros(out, vnx_sample->topic, 10);
}

void RelayBoardV3::handle(std::shared_ptr<const pilot::USBoardData> value){
	{
		auto out = std::make_shared<neo_msgs::USBoardV2>();
		out->header.stamp = pilot_to_ros_time(value->time);
		for(size_t i=0; i<std::min(out->sensor.size(), value->sensor.size()); i++){
			out->sensor[i] = value->sensor[i];
		}
		for(size_t i=0; i<std::min(out->analog.size(), value->analog_input.size()); i++){
			out->analog[i] = value->analog_input[i];
		}
		publish_to_ros(out, vnx_sample->topic, 10);
	}
	for(size_t i=0; i<value->sensor.size(); i++){
		const std::string key = "us_sensor_" + std::to_string(i);
		auto find = topics_to_ros.find(key);
		if(find != topics_to_ros.end()){
			auto out = std::make_shared<sensor_msgs::Range>();
			out->header.stamp = pilot_to_ros_time(value->time);
			out->header.frame_id = "us_" + std::to_string(i + 1) + "_link";
			out->radiation_type = sensor_msgs::Range::ULTRASOUND;
			out->field_of_view = 1.05;
			out->min_range = 0.1;
			out->max_range = 1.2;
			out->range = value->sensor[i];
			publish_to_ros(out, find->second, 10);
		}
	}
}

template<class T>
void RelayBoardV3::bulk_subscribe(std::function<void(std::shared_ptr<const T>, vnx::TopicPtr)> func, const std::map<std::string, vnx::TopicPtr> &mapping, int queue_size){
	for(const auto &entry : mapping){
		const auto &ros_topic = entry.first;
		const auto &pilot_topic = entry.second;
		if(!import_subscribers.count(ros_topic)){
			std::function<void(const std::shared_ptr<const T>&)> callback = std::bind(func, std::placeholders::_1, pilot_topic);
			std::shared_ptr<ros::Subscriber> subs(new ros::Subscriber(
				nh->subscribe<T>(ros_topic, queue_size, callback)
			));
			import_subscribers[ros_topic] = subs;
		}
	}
}

template<class T>
void RelayBoardV3::publish_to_ros(std::shared_ptr<T> sample, const std::string &ros_topic, int queue_size){
	auto &publisher = export_publishers[ros_topic];
	if(!publisher){
		std::shared_ptr<ros::Publisher> pub(new ros::Publisher(
			nh->advertise<T>(ros_topic, queue_size)
		));
		publisher = pub;
	}
	publisher->publish(*sample);
}

template<class T>
void RelayBoardV3::publish_to_ros(std::shared_ptr<T> sample, vnx::TopicPtr pilot_topic, int queue_size){
	auto find = topics_board_to_ros.find(pilot_topic);
	if(find == topics_board_to_ros.end()){
		throw std::logic_error("No export set for topic " + vnx::to_string(pilot_topic));
	}
	publish_to_ros(sample, find->second, queue_size);
}

void RelayBoardV3::init_board(){
	if(board_initialized){
		return;
	}
	module_launcher->launch(remote_config);
	board_initialized = true;
}

void RelayBoardV3::update(){
	const auto now = vnx::get_time_micros();
	for(auto iter = incident_map.begin(); iter != incident_map.end(); /* no iter */) {
		auto value = iter->second;
		if(value->timeout_ms > 0 && (now - value->time) / 1000 > value->timeout_ms) {
			auto copy = vnx::clone(value);
			copy->time = now;
			copy->is_active = false;
			copy->is_cleared = true;
			on_incident(copy);
			iter = incident_map.erase(iter);
		} else {
			iter++;
		}
	}
}

void RelayBoardV3::on_incident(std::shared_ptr<const pilot::Incident> value){
	int level = WARN;
	switch(value->event.type) {
		case pilot::event_type_e::ERROR: level = ERROR; break;
		case pilot::event_type_e::WARNING: level = WARN; break;
		case pilot::event_type_e::NOTIFICATION: level = INFO; break;
	}
	if(value->is_cleared) {
		level = INFO;
	}
	log(level) << value->to_log_message();
}

void RelayBoardV3::handle_KinematicsState(const neo_msgs::KinematicsState::ConstPtr& state, vnx::TopicPtr pilot_topic){
	auto out = pilot::kinematics::KinematicsState::create();
	out->time = vnx::get_time_micros();
	out->is_vel_cmd = state->is_vel_cmd;
	out->is_moving = state->is_moving;
	publish(out, pilot_topic);
}

void RelayBoardV3::handle_JointTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr& trajectory, vnx::TopicPtr pilot_topic) {
	if (trajectory->points.empty()) {
		log(WARN) << "Received JointTrajectory with no points.";
		return;
	}

	const auto& point = trajectory->points[0];

	if (kinematic_type == kinematic_type_e::DIFFERENTIAL) {
		auto out = pilot::kinematics::differential::DriveCmd::create();
		out->time = vnx::get_time_micros();

		for (size_t i = 0; i < std::min(trajectory->joint_names.size(), point.velocities.size()); ++i) {
			const auto& name = trajectory->joint_names[i];
			double v = point.velocities[i];

			if (name == "wheel_left_joint") {
				out->velocity.left = v;
			} else if (name == "wheel_right_joint") {
				out->velocity.right = v;
			} else {
				throw std::logic_error("Unknown joint name: " + name);
			}
		}
		publish(out, pilot_topic);
	}else if(kinematic_type == kinematic_type_e::MECANUM){
		auto out = pilot::kinematics::mecanum::DriveCmd::create();
		out->time = vnx::get_time_micros();

		for (size_t i = 0; i < std::min(trajectory->joint_names.size(), point.velocities.size()); ++i) {
			const auto& name = trajectory->joint_names[i];
			double v = point.velocities[i];

			if (name == "wheel_front_left_base_link") {
				out->velocity.front_left = v;
			} else if (name == "wheel_front_right_base_link") {
				out->velocity.front_right = v;
			} else if (name == "wheel_back_left_base_link") {
				out->velocity.back_left = v;
			} else if (name == "wheel_back_right_base_link") {
				out->velocity.back_right = v;
			} else {
				throw std::logic_error("Unknown joint name: " + name);
			}
		}

		publish(out, pilot_topic);

	} else if (kinematic_type == kinematic_type_e::OMNIDRIVE) {
		auto out = pilot::kinematics::omnidrive::DriveCmd::create();
		out->time = vnx::get_time_micros();

		size_t n = std::min({ trajectory->joint_names.size(), point.positions.size(), point.velocities.size() });

		for (size_t i = 0; i < n; ++i) {
			const auto& name = trajectory->joint_names[i];
			double p = point.positions[i];
			double v = point.velocities[i];

			using pos = pilot::kinematics::position_code_e;

			if (name == "wheel_front_left_joint") {
				out->drive_vel.set(pos::FRONT_LEFT, v);
			} else if (name == "wheel_front_right_joint") {
				out->drive_vel.set(pos::FRONT_RIGHT, v);
			} else if (name == "wheel_back_left_joint") {
				out->drive_vel.set(pos::BACK_LEFT, v);
			} else if (name == "wheel_back_right_joint") {
				out->drive_vel.set(pos::BACK_RIGHT, v);
			} else if (name == "wheel_front_joint") {
				out->drive_vel.set(pos::FRONT, v);
			} else if (name == "wheel_back_joint") {
				out->drive_vel.set(pos::BACK, v);
			} else if (name == "caster_front_left_joint") {
				out->steer_pos.set(pos::FRONT_LEFT, p);
			} else if (name == "caster_front_right_joint") {
				out->steer_pos.set(pos::FRONT_RIGHT, p);
			} else if (name == "caster_back_left_joint") {
				out->steer_pos.set(pos::BACK_LEFT, p);
			} else if (name == "caster_back_right_joint") {
				out->steer_pos.set(pos::BACK_RIGHT, p);
			} else if (name == "caster_front_joint") {
				out->steer_pos.set(pos::FRONT, p);
			} else if (name == "caster_back_joint") {
				out->steer_pos.set(pos::BACK, p);
			} else {
				throw std::logic_error("Unknown joint name: " + name);
			}
		}
		publish(out, pilot_topic);
	}else{
		throw std::logic_error("Unsupported kinematic type");
	}
}


bool RelayBoardV3::service_set_relay(neo_srvs::RelayBoardSetRelay::Request &req, neo_srvs::RelayBoardSetRelay::Response &res) {
	try{
		platform_interface->set_relay(req.id, req.state);
		res.success = true;
		return true;
	}catch(const std::exception &err){
		ROS_WARN("Service call failed with: %s", err.what());
		res.success = false;
		return false;
	}
}

bool RelayBoardV3::service_set_digital_output(neo_srvs::IOBoardSetDigOut::Request &req, neo_srvs::IOBoardSetDigOut::Response &res) {
	try{
		platform_interface->set_digital_output(req.id, req.state);
		res.success = true;
		return true;
	}catch(const std::exception &err){
		ROS_WARN("Service call failed with: %s", err.what());
		res.success = false;
		return false;
	}
}

bool RelayBoardV3::service_start_charging(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
	try{
		platform_interface->start_charging();
		return true;
	}catch(const std::exception &err){
		ROS_WARN("Service call failed with: %s", err.what());
		return false;
	}
}

bool RelayBoardV3::service_stop_charging(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
	try{
		platform_interface->stop_charging();
		return true;
	}catch(const std::exception &err){
		ROS_WARN("Service call failed with: %s", err.what());
		return false;
	}
}

bool RelayBoardV3::service_shutdown_platform(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
	try{
		platform_interface->shutdown();
		return true;
	}catch(const std::exception &err){
		ROS_WARN("Service call failed with: %s", err.what());
		return false;
	}
}

bool RelayBoardV3::service_set_safety_field(neo_srvs::SetSafetyField::Request &req, neo_srvs::SetSafetyField::Response &res) {
	try{
		safety_interface->select_safety_field(req.field_id);
		res.success = true;
		return true;
	}catch(const std::exception &err){
		ROS_WARN("Service call failed with: %s", err.what());
		res.success = false;
		return false;
	}
}

bool RelayBoardV3::service_set_safety_mode(neo_srvs::RelayBoardSetSafetyMode::Request &req, neo_srvs::RelayBoardSetSafetyMode::Response &res) {
	pilot::safety_mode_e mode = pilot::safety_mode_e::NONE;
	switch(req.set_safety_mode.mode) {
		case neo_msgs::SafetyMode::SM_NONE:
			mode = pilot::safety_mode_e::NONE;
			break;
		case neo_msgs::SafetyMode::SM_APPROACHING:
			mode = pilot::safety_mode_e::APPROACHING;
			break;
		case neo_msgs::SafetyMode::SM_DEPARTING:
			mode = pilot::safety_mode_e::DEPARTING;
			break;
		case neo_msgs::SafetyMode::SM_WORKING:
			mode = pilot::safety_mode_e::WORKING;
			break;
		case neo_msgs::SafetyMode::SM_HANDLING:
			mode = pilot::safety_mode_e::HANDLING;
			break;
	}
	try{
		safety_interface->set_safety_mode(mode, req.station);
		res.success = true;
		return true;
	}catch(const std::exception &err){
		ROS_WARN("Service call failed with: %s", err.what());
		res.success = false;
		return false;
	}
}

bool RelayBoardV3::service_set_leds(neo_srvs::RelayBoardSetLED::Request &req, neo_srvs::RelayBoardSetLED::Response &res) {
	try{
		std::map<led_color_e, double> led_state;
		led_state[led_color_e::RED] = req.led_state.r;
		led_state[led_color_e::GREEN] = req.led_state.g;
		led_state[led_color_e::BLUE] = req.led_state.b;
		platform_interface->set_leds(led_state);
		res.success = true;
		return true;
	}catch(const std::exception &err){
		ROS_WARN("Service call failed with: %s", err.what());
		res.success = false;
		return false;
	}
}

} // neo_relayboard_v3
 