
// AUTO GENERATED by vnxcppcodegen

#ifndef INCLUDE_neo_relayboard_v3_RelayBoardV3_CLIENT_HXX_
#define INCLUDE_neo_relayboard_v3_RelayBoardV3_CLIENT_HXX_

#include <vnx/Client.h>
#include <pilot/BatteryState.hxx>
#include <pilot/EmergencyState.hxx>
#include <pilot/IOBoardData.hxx>
#include <pilot/Incident.hxx>
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

class RelayBoardV3Client : public vnx::Client {
public:
	RelayBoardV3Client(const std::string& service_name);
	
	RelayBoardV3Client(vnx::Hash64 service_addr);
	
	::vnx::Object vnx_get_config_object();
	
	::vnx::Variant vnx_get_config(const std::string& name = "");
	
	void vnx_set_config_object(const ::vnx::Object& config = ::vnx::Object());
	
	void vnx_set_config_object_async(const ::vnx::Object& config = ::vnx::Object());
	
	void vnx_set_config(const std::string& name = "", const ::vnx::Variant& value = ::vnx::Variant());
	
	void vnx_set_config_async(const std::string& name = "", const ::vnx::Variant& value = ::vnx::Variant());
	
	::vnx::TypeCode vnx_get_type_code();
	
	std::shared_ptr<const ::vnx::ModuleInfo> vnx_get_module_info();
	
	void vnx_restart();
	
	void vnx_restart_async();
	
	void vnx_stop();
	
	void vnx_stop_async();
	
	vnx::bool_t vnx_self_test();
	
};


} // namespace neo_relayboard_v3

#endif // INCLUDE_neo_relayboard_v3_RelayBoardV3_CLIENT_HXX_
