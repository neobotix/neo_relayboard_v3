/*
 * RelayBoardV3.cpp
 *
 *  Created on: Jan 19, 2022
 *      Author: jaw
 */

#include <pilot/relayboardv3/RelayBoardV3.h>
#include <pilot/kinematics/differential/DriveCmd.hxx>
#include <pilot/kinematics/mecanum/DriveCmd.hxx>
#include <pilot/kinematics/omnidrive/DriveCmd.hxx>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>


namespace pilot{
namespace relayboardv3{


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
			log(ERROR) << "Unsupported ROS type: " << type;
		}
	}

	Super::main();
	ros::shutdown();
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

} // relayboardv3
} // pilot



