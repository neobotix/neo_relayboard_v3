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

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_srvs/Empty.h>
#include <neo_srvs/RelayBoardSetRelay.h>
#include <neo_srvs/IOBoardSetDigOut.h>
#include <neo_srvs/RelayBoardSetLCDMsg.h>
#include <neo_srvs/RelayBoardSetSafetyMode.h>
#include <neo_srvs/SetSafetyField.h>
#include <neo_srvs/RelayBoardSetLED.h>
#include <neo_msgs/RelayBoardV3.h>
#include <neo_msgs/SafetyState.h>
#include <neo_msgs/KinematicsState.h>

namespace neo_relayboard_v3{

class RelayBoardV3 : public RelayBoardV3Base{
public:
    RelayBoardV3(const std::string &_vnx_name, ros::NodeHandle& node_handle);

protected:
    void main() override;

    void handle(std::shared_ptr<const vnx::LogMsg> value) override;
    void handle(std::shared_ptr<const pilot::Incident> value) override;
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

    void handle_JointTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr& trajectory, vnx::TopicPtr pilot_topic);
    void handle_KinematicsState(const neo_msgs::KinematicsState::ConstPtr& state, vnx::TopicPtr pilot_topic);

private:
    ros::NodeHandle* nh;
    std::map<std::string, ros::Subscriber> import_subscribers;
    std::map<std::string, std::shared_ptr<ros::Publisher>> export_publishers;

    std::shared_ptr<pilot::PlatformInterfaceClient> platform_interface;
    std::shared_ptr<pilot::SafetyInterfaceClient> safety_interface;
    std::shared_ptr<pilot::ModuleLauncherClient> module_launcher;

    bool board_initialized = false;
    std::map<std::tuple<std::string, std::string, std::string>, std::shared_ptr<const pilot::Incident>> incident_map;
    std::shared_ptr<const pilot::PowerState> m_power_state;
    std::vector<std::string> m_system_error;

    ros::ServiceServer srv_set_relay;
    ros::ServiceServer srv_set_digital_output;
    ros::ServiceServer srv_start_charging;
    ros::ServiceServer srv_stop_charging;
    ros::ServiceServer srv_shutdown_platform;
    ros::ServiceServer srv_set_safety_field;
    ros::ServiceServer srv_set_safety_mode;
    ros::ServiceServer srv_set_leds;

    template<class T>
    void bulk_subscribe(std::function<void(std::shared_ptr<const T>, vnx::TopicPtr)> func, const std::map<std::string, vnx::TopicPtr> &mapping, int queue_size);
    template<class T>
    void publish_to_ros(std::shared_ptr<T> sample, const std::string &ros_topic, int queue_size);
    template<class T>
    void publish_to_ros(std::shared_ptr<T> sample, vnx::TopicPtr pilot_topic, int queue_size);

    void init_board();
    void update();
    void on_incident(std::shared_ptr<const pilot::Incident> value);

    bool is_shutdown = false;

    bool service_set_relay(neo_srvs::RelayBoardSetRelay::Request &req, neo_srvs::RelayBoardSetRelay::Response &res);
    bool service_set_digital_output(neo_srvs::IOBoardSetDigOut::Request &req, neo_srvs::IOBoardSetDigOut::Response &res);
    bool service_start_charging(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool service_stop_charging(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool service_shutdown_platform(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool service_set_safety_field(neo_srvs::SetSafetyField::Request &req, neo_srvs::SetSafetyField::Response &res);
    bool service_set_safety_mode(neo_srvs::RelayBoardSetSafetyMode::Request &req, neo_srvs::RelayBoardSetSafetyMode::Response &res);
    bool service_set_leds(neo_srvs::RelayBoardSetLED::Request &req, neo_srvs::RelayBoardSetLED::Response &res);
};


} // neo_relayboard_v3


#endif /* INCLUDE_neo_relayboard_v3_RelayBoardV3_H */
