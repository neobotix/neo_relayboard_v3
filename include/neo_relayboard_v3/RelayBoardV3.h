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

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_srvs/Empty.h>
#include <neo_srvs/RelayBoardSetRelay.h>
#include <neo_srvs/IOBoardSetDigOut.h>
#include <neo_srvs/RelayBoardSetLCDMsg.h>


namespace neo_relayboard_v3{


class RelayBoardV3 : public RelayBoardV3Base{
public:
	RelayBoardV3(const std::string &_vnx_name);

protected:
	void init() override;
	void main() override;

	void handle(std::shared_ptr<const pilot::SystemState> value) override;
	void handle(std::shared_ptr<const pilot::EmergencyState> value) override;
	void handle(std::shared_ptr<const pilot::BatteryState> value) override;
	void handle(std::shared_ptr<const pilot::PowerState> value) override;
	void handle(std::shared_ptr<const pilot::kinematics::differential::DriveState> value) override;
	void handle(std::shared_ptr<const pilot::kinematics::mecanum::DriveState> value) override;
	void handle(std::shared_ptr<const pilot::kinematics::omnidrive::DriveState> value) override;
	void handle(std::shared_ptr<const pilot::RelayBoardData> value) override;
	void handle(std::shared_ptr<const pilot::IOBoardData> value) override;
	void handle(std::shared_ptr<const pilot::USBoardData> value) override;

private:
	ros::NodeHandle nh;
	std::map<std::string, ros::Publisher> export_publishers;
	std::vector<ros::Subscriber> ros_subscriptions;
	std::vector<ros::ServiceServer> ros_services;
	std::shared_ptr<pilot::PlatformInterfaceClient> platform_interface;
	std::shared_ptr<const pilot::PowerState> m_power_state;

	template<class T>
	void bulk_subscribe(std::function<void(const typename T::ConstPtr&, vnx::TopicPtr)> func, const std::map<std::string, vnx::TopicPtr> mapping){
		for(const auto &entry : mapping){
			const auto &ros_topic = entry.first;
			const auto &pilot_topic = entry.second;
			auto sub = nh.subscribe<T>(ros_topic, max_subscribe_queue_ros, boost::bind(func, _1, pilot_topic));
			ros_subscriptions.push_back(sub);
		}
	}

	template<class T>
	void publish_to_ros(boost::shared_ptr<T> sample, const std::string &ros_topic){
		const size_t size_before = export_publishers.size();
		auto &publisher = export_publishers[ros_topic];
		if(export_publishers.size() > size_before){
			publisher = nh.advertise<T>(ros_topic, max_publish_queue_ros);
		}
		publisher.publish(sample);
	}

	template<class T>
	void publish_to_ros(boost::shared_ptr<T> sample, vnx::TopicPtr pilot_topic){
		auto find = topics_board_to_ros.find(pilot_topic);
		if(find == topics_board_to_ros.end()){
			throw std::logic_error("No export set for topic " + vnx::to_string(pilot_topic));
		}
		publish_to_ros(sample, find->second);
	}

	void handle_JointTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr &trajectory, vnx::TopicPtr pilot_topic);

	bool service_set_relay(neo_srvs::RelayBoardSetRelay::Request &req, neo_srvs::RelayBoardSetRelay::Response &res);
	bool service_set_digital_output(neo_srvs::IOBoardSetDigOut::Request &req, neo_srvs::IOBoardSetDigOut::Response &res);
	bool service_start_charging(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
	bool service_stop_charging(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
	bool service_set_LCD_message(neo_srvs::RelayBoardSetLCDMsg::Request &req, neo_srvs::RelayBoardSetLCDMsg::Response &res);
};


} // neo_relayboard_v3


#endif /* INCLUDE_pilot_relayboardv3_RelayBoardV3_H */

