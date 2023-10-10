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

#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <neo_msgs2/msg/emergency_stop_state.hpp>
#include <neo_msgs2/msg/io_board.hpp>
#include <neo_msgs2/msg/us_board_v2.hpp>

#include <cmath>


namespace neo_relayboard_v3{

using namespace pilot;

inline
rclcpp::Time pilot_to_ros_time(const int64_t& time_usec)
{
	rclcpp::Time time(time_usec * 1000);
	return time;
}


RelayBoardV3::RelayBoardV3(const std::string &_vnx_name, std::shared_ptr<rclcpp::Node> node_handle):
	RelayBoardV3Base(_vnx_name)
{
	nh = node_handle;
}


void RelayBoardV3::main(){
	for(const auto &entry : topics_board_to_ros){
		subscribe(entry.first, 100);
	}
	for(const auto &entry : topics_ros_to_board){
		const auto &ros_type = entry.first;
		if(ros_type == "trajectory_msgs/JointTrajectory"){
			bulk_subscribe<trajectory_msgs::msg::JointTrajectory>(std::bind(&RelayBoardV3::handle_JointTrajectory, this, std::placeholders::_1, std::placeholders::_2), entry.second, rclcpp::QoS(rclcpp::KeepLast(max_subscribe_queue_ros)));
		}else{
			log(WARN) << "Unsupported ROS type: " << ros_type;
		}
	}
	for(const auto &topic : topics_from_board){
		subscribe(topic, 100);
	}
	platform_interface = std::make_shared<PlatformInterfaceClient>(platform_interface_server);
	safety_interface = std::make_shared<SafetyInterfaceClient>(safety_server);
	module_launcher = std::make_shared<ModuleLauncherClient>(launcher_server);

	srv_set_relay = nh->create_service<neo_srvs2::srv::RelayBoardSetRelay>("set_relay", std::bind(&RelayBoardV3::service_set_relay, this, std::placeholders::_1, std::placeholders::_2));
	srv_io_board_set_dig_out = nh->create_service<neo_srvs2::srv::IOBoardSetDigOut>("/ioboard/set_digital_output", std::bind(&RelayBoardV3::service_set_digital_output, this, std::placeholders::_1, std::placeholders::_2));
	srv_start_charging = nh->create_service<std_srvs::srv::Empty>("start_charging", std::bind(&RelayBoardV3::service_start_charging, this, std::placeholders::_1, std::placeholders::_2));
	srv_stop_charging = nh->create_service<std_srvs::srv::Empty>("stop_charging", std::bind(&RelayBoardV3::service_stop_charging, this, std::placeholders::_1, std::placeholders::_2));
	srv_set_LCD_message = nh->create_service<neo_srvs2::srv::RelayBoardSetLCDMsg>("set_LCD_msg", std::bind(&RelayBoardV3::service_set_LCD_message, this, std::placeholders::_1, std::placeholders::_2));

	if(board_init_interval_ms > 0){
		set_timer_millis(board_init_interval_ms, std::bind(&RelayBoardV3::init_board, this));
	}

	Super::main();
	rclcpp::shutdown();
}


void RelayBoardV3::handle(std::shared_ptr<const pilot::SystemState> value){
	// TODO
}


void RelayBoardV3::handle(std::shared_ptr<const pilot::SafetyState> value){
	// TODO
}


void RelayBoardV3::handle(std::shared_ptr<const pilot::EmergencyState> value){
	auto out = std::make_shared<neo_msgs2::msg::EmergencyStopState>();
	out->header.stamp = pilot_to_ros_time(value->time);

	out->emergency_button_stop = false;
	out->scanner_stop = false;
	if(value->code == safety_code_e::EMERGENCY_STOP){
		out->emergency_button_stop = true;
	}else if(value->code == safety_code_e::SCANNER_STOP){
		out->scanner_stop = true;
	}

	if(value->state == em_stop_state_e::STOPPED){
		out->emergency_state = neo_msgs2::msg::EmergencyStopState::EMSTOP;
	}else if(value->state == em_stop_state_e::CONFIRMED){
		out->emergency_state = neo_msgs2::msg::EmergencyStopState::EMCONFIRMED;
	}else{
		out->emergency_state = neo_msgs2::msg::EmergencyStopState::EMFREE;
	}

	publish_to_ros(out, vnx_sample->topic, rclcpp::QoS(rclcpp::KeepLast(max_publish_queue_ros)));
}


void RelayBoardV3::handle(std::shared_ptr<const pilot::BatteryState> value){
	auto out = std::make_shared<sensor_msgs::msg::BatteryState>();
	out->header.stamp = pilot_to_ros_time(value->time);

	out->voltage = value->voltage;
	out->current = value->current;
	out->charge = NAN;
	out->capacity = NAN;
	out->design_capacity = NAN;
	out->percentage = value->remaining;
	out->power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
	if(m_power_state){
		auto chrg = m_power_state->charging_state;
		if(chrg == charging_state_e::IS_CHARGING){
			out->power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
		}else if(chrg == charging_state_e::FINISHED){
			out->power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
		}else{
			out->power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
	}
	out->power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
	out->power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
	if(value->type == battery_type_e::AGM){
		// lead
	}else if(value->type == battery_type_e::LFP){
		out->power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIFE;
	}
	out->present = true;

	publish_to_ros(out, vnx_sample->topic, rclcpp::QoS(rclcpp::KeepLast(max_publish_queue_ros)));
}


void RelayBoardV3::handle(std::shared_ptr<const pilot::PowerState> value){
	// to be merged with BatteryState
	m_power_state = value;
}


void RelayBoardV3::handle(std::shared_ptr<const pilot::kinematics::bicycle::DriveState> value){
	const std::string dont_optimize_away_the_library = vnx::to_string(*value);
	// TODO
}


void RelayBoardV3::handle(std::shared_ptr<const pilot::kinematics::differential::DriveState> value){
	const std::string dont_optimize_away_the_library = vnx::to_string(*value);
	auto out = std::make_shared<sensor_msgs::msg::JointState>();
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
	publish_to_ros(out, vnx_sample->topic, rclcpp::QoS(rclcpp::KeepLast(max_publish_queue_ros)));
}


void RelayBoardV3::handle(std::shared_ptr<const pilot::kinematics::mecanum::DriveState> value){
	const std::string dont_optimize_away_the_library = vnx::to_string(*value);
	auto out = std::make_shared<sensor_msgs::msg::JointState>();
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
	publish_to_ros(out, vnx_sample->topic, rclcpp::QoS(rclcpp::KeepLast(max_publish_queue_ros)));
}


void RelayBoardV3::handle(std::shared_ptr<const pilot::kinematics::omnidrive::DriveState> value){
	const std::string dont_optimize_away_the_library = vnx::to_string(*value);
	auto out = std::make_shared<sensor_msgs::msg::JointState>();
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
	out->position[0] = value->drive_pos.get(pilot::kinematics::position_code_e::FRONT_LEFT);
	out->position[1] = value->steer_pos.get(pilot::kinematics::position_code_e::FRONT_LEFT);
	out->position[2] = value->drive_pos.get(pilot::kinematics::position_code_e::BACK_LEFT);
	out->position[3] = value->steer_pos.get(pilot::kinematics::position_code_e::BACK_LEFT);
	out->position[4] = value->drive_pos.get(pilot::kinematics::position_code_e::BACK_RIGHT);
	out->position[5] = value->steer_pos.get(pilot::kinematics::position_code_e::BACK_RIGHT);
	out->position[6] = value->drive_pos.get(pilot::kinematics::position_code_e::FRONT_RIGHT);
	out->position[7] = value->steer_pos.get(pilot::kinematics::position_code_e::FRONT_RIGHT);
	out->velocity[0] = value->drive_vel.get(pilot::kinematics::position_code_e::FRONT_LEFT);
	out->velocity[1] = value->steer_vel.get(pilot::kinematics::position_code_e::FRONT_LEFT);
	out->velocity[2] = value->drive_vel.get(pilot::kinematics::position_code_e::BACK_LEFT);
	out->velocity[3] = value->steer_vel.get(pilot::kinematics::position_code_e::BACK_LEFT);
	out->velocity[4] = value->drive_vel.get(pilot::kinematics::position_code_e::BACK_RIGHT);
	out->velocity[5] = value->steer_vel.get(pilot::kinematics::position_code_e::BACK_RIGHT);
	out->velocity[6] = value->drive_vel.get(pilot::kinematics::position_code_e::FRONT_RIGHT);
	out->velocity[7] = value->steer_vel.get(pilot::kinematics::position_code_e::FRONT_RIGHT);
	if(value->has_torque) {
		out->effort.resize(8);
		out->effort[0] = value->drive_torque.get(pilot::kinematics::position_code_e::FRONT_LEFT);
		out->effort[1] = value->steer_torque.get(pilot::kinematics::position_code_e::FRONT_LEFT);
		out->effort[2] = value->drive_torque.get(pilot::kinematics::position_code_e::BACK_LEFT);
		out->effort[3] = value->steer_torque.get(pilot::kinematics::position_code_e::BACK_LEFT);
		out->effort[4] = value->drive_torque.get(pilot::kinematics::position_code_e::BACK_RIGHT);
		out->effort[5] = value->steer_torque.get(pilot::kinematics::position_code_e::BACK_RIGHT);
		out->effort[6] = value->drive_torque.get(pilot::kinematics::position_code_e::FRONT_RIGHT);
		out->effort[7] = value->steer_torque.get(pilot::kinematics::position_code_e::FRONT_RIGHT);
	}
	publish_to_ros(out, vnx_sample->topic, rclcpp::QoS(rclcpp::KeepLast(max_publish_queue_ros)));
}


void RelayBoardV3::handle(std::shared_ptr<const pilot::RelayBoardV3Data> value){
	// TODO
}


void RelayBoardV3::handle(std::shared_ptr<const pilot::IOBoardData> value){
	auto out = std::make_shared<neo_msgs2::msg::IOBoard>();
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

	publish_to_ros(out, vnx_sample->topic, rclcpp::QoS(rclcpp::KeepLast(max_publish_queue_ros)));
}


void RelayBoardV3::handle(std::shared_ptr<const pilot::USBoardData> value){
	{
		auto out = std::make_shared<neo_msgs2::msg::USBoardV2>();
		out->header.stamp = pilot_to_ros_time(value->time);

		for(size_t i=0; i<std::min(out->sensor.size(), value->sensor.size()); i++){
			out->sensor[i] = value->sensor[i];
		}
		for(size_t i=0; i<std::min(out->analog.size(), value->analog_input.size()); i++){
			out->analog[i] = value->analog_input[i];
		}
		publish_to_ros(out, vnx_sample->topic, rclcpp::QoS(rclcpp::KeepLast(max_publish_queue_ros)));
	}

	for(size_t i=0; i<value->sensor.size(); i++){
		const std::string key = "us_sensor_" + std::to_string(i);
		auto find = topics_to_ros.find(key);
		if(find != topics_to_ros.end()){
			auto out = std::make_shared<sensor_msgs::msg::Range>();
			out->header.stamp = pilot_to_ros_time(value->time);
			out->header.frame_id = "us_" + std::to_string(i + 1) + "_link";
			out->radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
			out->field_of_view = 1.05;
			out->min_range = 0.1;
			out->max_range = 1.2;
			out->range = value->sensor[i];
			publish_to_ros(out, find->second, rclcpp::QoS(rclcpp::KeepLast(max_publish_queue_ros)));
		}
	}
}


template<class T>
void RelayBoardV3::bulk_subscribe(std::function<void(std::shared_ptr<const T>, vnx::TopicPtr)> func, const std::map<std::string, vnx::TopicPtr> &mapping, const rclcpp::QoS &qos){
	for(const auto &entry : mapping){
		const auto &ros_topic = entry.first;
		const auto &pilot_topic = entry.second;
		if(!import_subscribers.count(ros_topic)){
			std::function<void(std::shared_ptr<const T>)> callback = std::bind(func, std::placeholders::_1, pilot_topic);
			std::shared_ptr<rclcpp::SubscriptionBase> subs = nh->create_subscription<T>(ros_topic, qos, callback);
			import_subscribers[ros_topic] = subs;
		}
	}
}


template<class T>
void RelayBoardV3::publish_to_ros(std::shared_ptr<T> sample, const std::string &ros_topic, const rclcpp::QoS &qos){
	const size_t size_before = export_publishers.size();
	auto &publisher = export_publishers[ros_topic];
	if(!publisher){
		publisher = nh->create_publisher<T>(ros_topic, qos);
	}
	if(auto publisher_ = std::dynamic_pointer_cast<rclcpp::Publisher<T>>(publisher)){
		publisher_->publish(*sample);
	}
}


template<class T>
void RelayBoardV3::publish_to_ros(std::shared_ptr<T> sample, vnx::TopicPtr pilot_topic, const rclcpp::QoS &qos){
	auto find = topics_board_to_ros.find(pilot_topic);
	if(find == topics_board_to_ros.end()){
		throw std::logic_error("No export set for topic " + vnx::to_string(pilot_topic));
	}
	publish_to_ros(sample, find->second, qos);
}


void RelayBoardV3::init_board(){
	if(board_initialized){
		return;
	}

	module_launcher->launch(remote_config);
	board_initialized = true;
}


void RelayBoardV3::handle_JointTrajectory(std::shared_ptr<const trajectory_msgs::msg::JointTrajectory> trajectory, vnx::TopicPtr pilot_topic){
	const trajectory_msgs::msg::JointTrajectoryPoint &point = trajectory->points[0];

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
				out->drive_vel.set(pilot::kinematics::position_code_e::FRONT_LEFT, v);
			}else if(name == "mpo_700_wheel_front_right_joint"){
				out->drive_vel.set(pilot::kinematics::position_code_e::FRONT_RIGHT, v);
			}else if(name == "mpo_700_wheel_back_left_joint"){
				out->drive_vel.set(pilot::kinematics::position_code_e::BACK_LEFT, v);
			}else if(name == "mpo_700_wheel_back_right_joint"){
				out->drive_vel.set(pilot::kinematics::position_code_e::BACK_RIGHT, v);
			}else if(name == "mpo_700_caster_front_left_joint"){
				out->drive_vel.set(pilot::kinematics::position_code_e::FRONT_LEFT, v);
			}else if(name == "mpo_700_caster_front_right_joint"){
				out->drive_vel.set(pilot::kinematics::position_code_e::FRONT_RIGHT, v);
			}else if(name == "mpo_700_caster_back_left_joint"){
				out->drive_vel.set(pilot::kinematics::position_code_e::BACK_LEFT, v);
			}else if(name == "mpo_700_caster_back_right_joint"){
				out->drive_vel.set(pilot::kinematics::position_code_e::BACK_RIGHT, v);
			}else{
				throw std::logic_error("Unknown joint name: " + name);
			}
		}
		publish(out, pilot_topic);
	}else{
		throw std::logic_error("Unsupported kinematic type");
	}
}


bool RelayBoardV3::service_set_relay(std::shared_ptr<neo_srvs2::srv::RelayBoardSetRelay::Request> req, std::shared_ptr<neo_srvs2::srv::RelayBoardSetRelay::Response> res){
	try{
		platform_interface->set_relay(req->id, req->state);
		res->success = true;
		return true;
	}catch(const std::exception &err){
		const std::string error = "Service call failed with: " + std::string(err.what());
		RCLCPP_ERROR_STREAM(nh->get_logger(), error);
		res->success = false;
		return false;
	}
}


bool RelayBoardV3::service_set_digital_output(std::shared_ptr<neo_srvs2::srv::IOBoardSetDigOut::Request> req, std::shared_ptr<neo_srvs2::srv::IOBoardSetDigOut::Response> res){
	try{
		platform_interface->set_digital_output(req->id, req->state);
		res->success = true;
		return true;
	}catch(const std::exception &err){
		const std::string error = "Service call failed with: " + std::string(err.what());
		RCLCPP_ERROR_STREAM(nh->get_logger(), error);
		res->success = false;
		return false;
	}
}


bool RelayBoardV3::service_start_charging(std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res){
	try{
		platform_interface->start_charging();
		return true;
	}catch(const std::exception &err){
		const std::string error = "Service call failed with: " + std::string(err.what());
		RCLCPP_ERROR_STREAM(nh->get_logger(), error);
		return false;
	}
}


bool RelayBoardV3::service_stop_charging(std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res){
	try{
		platform_interface->stop_charging();
		return true;
	}catch(const std::exception &err){
		const std::string error = "Service call failed with: " + std::string(err.what());
		RCLCPP_ERROR_STREAM(nh->get_logger(), error);
		return false;
	}
}


bool RelayBoardV3::service_set_LCD_message(std::shared_ptr<neo_srvs2::srv::RelayBoardSetLCDMsg::Request> req, std::shared_ptr<neo_srvs2::srv::RelayBoardSetLCDMsg::Response> res){
	try{
		platform_interface->set_display_text(req->message);
		res->success = true;
		return true;
	}catch(const std::exception &err){
		const std::string error = "Service call failed with: " + std::string(err.what());
		RCLCPP_ERROR_STREAM(nh->get_logger(), error);
		res->success = false;
		return false;
	}
}


} // neo_relayboard_v3



