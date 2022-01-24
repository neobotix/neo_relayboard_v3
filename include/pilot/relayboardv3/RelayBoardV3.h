/*
 * RelayBoardV3.h
 *
 *  Created on: Jan 19, 2022
 *      Author: jaw
 */

#ifndef INCLUDE_pilot_relayboardv3_RelayBoardV3_H
#define INCLUDE_pilot_relayboardv3_RelayBoardV3_H

#include <pilot/relayboardv3/RelayBoardV3Base.hxx>
#include <pilot/kinematics/differential/DriveState.hxx>
#include <pilot/kinematics/mecanum/DriveState.hxx>
#include <pilot/kinematics/omnidrive/DriveState.hxx>

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>


namespace pilot{
namespace relayboardv3{


class RelayBoardV3 : public RelayBoardV3Base{
public:
	RelayBoardV3(const std::string &_vnx_name);

protected:
	void init() override;
	void main() override;

	void handle(std::shared_ptr<const pilot::kinematics::differential::DriveState> value) override;
	void handle(std::shared_ptr<const pilot::kinematics::mecanum::DriveState> value) override;
	void handle(std::shared_ptr<const pilot::kinematics::omnidrive::DriveState> value) override;

private:
	ros::NodeHandle nh;
	std::map<std::string, ros::Publisher> export_publishers;

	template<class T>
	void bulk_subscribe(std::function<void(const typename T::ConstPtr&, vnx::TopicPtr)> func, const std::map<std::string, vnx::TopicPtr> mapping){
		for(const auto &entry : mapping){
			const auto &ros_topic = entry.first;
			const auto &pilot_topic = entry.second;
			nh.subscribe<T>(ros_topic, max_subscribe_queue_ros, boost::bind(func, _1, pilot_topic));
		}
	}

	template<class T>
	void publish_to_ros(boost::shared_ptr<T> sample, vnx::TopicPtr pilot_topic){
		auto find = topics_board_to_ros.find(pilot_topic);
		if(find == topics_board_to_ros.end()){
			throw std::logic_error("No export set for topic " + vnx::to_string(pilot_topic));
		}
		auto ros_topic = find->second;

		const size_t size_before = export_publishers.size();
		auto &publisher = export_publishers[ros_topic];
		if(export_publishers.size() > size_before){
			publisher = nh.advertise<T>(ros_topic, max_publish_queue_ros);
		}
		publisher.publish(sample);
	}

	void handle_JointTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr &trajectory, vnx::TopicPtr pilot_topic);

};


} // relayboardv3
} // pilot


#endif /* INCLUDE_pilot_relayboardv3_RelayBoardV3_H */

