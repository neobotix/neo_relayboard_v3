
// AUTO GENERATED by vnxcppcodegen

#ifndef INCLUDE_neo_relayboard_v3_RelayBoardV3Base_HXX_
#define INCLUDE_neo_relayboard_v3_RelayBoardV3Base_HXX_

#include <neo_relayboard_v3/package.hxx>
#include <pilot/BatteryState.hxx>
#include <pilot/EmergencyState.hxx>
#include <pilot/IOBoardData.hxx>
#include <pilot/PowerState.hxx>
#include <pilot/RelayBoardV3Data.hxx>
#include <pilot/SafetyState.hxx>
#include <pilot/SystemState.hxx>
#include <pilot/USBoardData.hxx>
#include <pilot/kinematic_type_e.hxx>
#include <pilot/kinematics/bicycle/DriveState.hxx>
#include <pilot/kinematics/differential/DriveState.hxx>
#include <pilot/kinematics/mecanum/DriveState.hxx>
#include <pilot/kinematics/omnidrive/DriveState.hxx>
#include <vnx/LogMsg.hxx>
#include <vnx/Module.h>
#include <vnx/Object.hpp>
#include <vnx/TopicPtr.hpp>


namespace neo_relayboard_v3 {

class RelayBoardV3Base : public ::vnx::Module {
public:
	
	std::map<::vnx::TopicPtr, std::string> topics_board_to_ros;
	std::map<std::string, std::map<std::string, ::vnx::TopicPtr>> topics_ros_to_board;
	std::vector<::vnx::TopicPtr> topics_from_board;
	std::map<std::string, std::string> topics_to_ros;
	std::string platform_interface_server = "PlatformInterface";
	std::string safety_server = "SafetyInterface";
	std::string launcher_server = "RelayBoardV3Launcher";
	int32_t board_init_interval_ms = 1500;
	::vnx::Object remote_config;
	int32_t max_publish_queue_ros = 1;
	int32_t max_subscribe_queue_ros = 1;
	::pilot::kinematic_type_e kinematic_type;
	
	typedef ::vnx::Module Super;
	
	static const vnx::Hash64 VNX_TYPE_HASH;
	static const vnx::Hash64 VNX_CODE_HASH;
	
	static constexpr uint64_t VNX_TYPE_ID = 0x59c86a8cb3f030daull;
	
	RelayBoardV3Base(const std::string& _vnx_name);
	
	vnx::Hash64 get_type_hash() const override;
	std::string get_type_name() const override;
	const vnx::TypeCode* get_type_code() const override;
	
	void read(std::istream& _in) override;
	void write(std::ostream& _out) const override;
	
	void accept(vnx::Visitor& _visitor) const override;
	
	vnx::Object to_object() const override;
	void from_object(const vnx::Object& object) override;
	
	vnx::Variant get_field(const std::string& name) const override;
	void set_field(const std::string& name, const vnx::Variant& value) override;
	
	friend std::ostream& operator<<(std::ostream& _out, const RelayBoardV3Base& _value);
	friend std::istream& operator>>(std::istream& _in, RelayBoardV3Base& _value);
	
	static const vnx::TypeCode* static_get_type_code();
	static std::shared_ptr<vnx::TypeCode> static_create_type_code();
	
protected:
	using Super::handle;
	
	virtual void handle(std::shared_ptr<const ::vnx::LogMsg> _value) {}
	virtual void handle(std::shared_ptr<const ::pilot::SystemState> _value) {}
	virtual void handle(std::shared_ptr<const ::pilot::SafetyState> _value) {}
	virtual void handle(std::shared_ptr<const ::pilot::EmergencyState> _value) {}
	virtual void handle(std::shared_ptr<const ::pilot::BatteryState> _value) {}
	virtual void handle(std::shared_ptr<const ::pilot::PowerState> _value) {}
	virtual void handle(std::shared_ptr<const ::pilot::kinematics::bicycle::DriveState> _value) {}
	virtual void handle(std::shared_ptr<const ::pilot::kinematics::differential::DriveState> _value) {}
	virtual void handle(std::shared_ptr<const ::pilot::kinematics::mecanum::DriveState> _value) {}
	virtual void handle(std::shared_ptr<const ::pilot::kinematics::omnidrive::DriveState> _value) {}
	virtual void handle(std::shared_ptr<const ::pilot::RelayBoardV3Data> _value) {}
	virtual void handle(std::shared_ptr<const ::pilot::IOBoardData> _value) {}
	virtual void handle(std::shared_ptr<const ::pilot::USBoardData> _value) {}
	
	void vnx_handle_switch(std::shared_ptr<const vnx::Value> _value) override;
	std::shared_ptr<vnx::Value> vnx_call_switch(std::shared_ptr<const vnx::Value> _method, const vnx::request_id_t& _request_id) override;
	
};


} // namespace neo_relayboard_v3


namespace vnx {

} // vnx

#endif // INCLUDE_neo_relayboard_v3_RelayBoardV3Base_HXX_
