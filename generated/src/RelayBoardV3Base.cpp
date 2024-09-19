
// AUTO GENERATED by vnxcppcodegen

#include <neo_relayboard_v3/package.hxx>
#include <neo_relayboard_v3/RelayBoardV3Base.hxx>
#include <vnx/NoSuchMethod.hxx>
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
#include <vnx/ModuleInterface_vnx_get_config.hxx>
#include <vnx/ModuleInterface_vnx_get_config_return.hxx>
#include <vnx/ModuleInterface_vnx_get_config_object.hxx>
#include <vnx/ModuleInterface_vnx_get_config_object_return.hxx>
#include <vnx/ModuleInterface_vnx_get_module_info.hxx>
#include <vnx/ModuleInterface_vnx_get_module_info_return.hxx>
#include <vnx/ModuleInterface_vnx_get_type_code.hxx>
#include <vnx/ModuleInterface_vnx_get_type_code_return.hxx>
#include <vnx/ModuleInterface_vnx_restart.hxx>
#include <vnx/ModuleInterface_vnx_restart_return.hxx>
#include <vnx/ModuleInterface_vnx_self_test.hxx>
#include <vnx/ModuleInterface_vnx_self_test_return.hxx>
#include <vnx/ModuleInterface_vnx_set_config.hxx>
#include <vnx/ModuleInterface_vnx_set_config_return.hxx>
#include <vnx/ModuleInterface_vnx_set_config_object.hxx>
#include <vnx/ModuleInterface_vnx_set_config_object_return.hxx>
#include <vnx/ModuleInterface_vnx_stop.hxx>
#include <vnx/ModuleInterface_vnx_stop_return.hxx>
#include <vnx/Object.hpp>
#include <vnx/TopicPtr.hpp>

#include <vnx/vnx.h>


namespace neo_relayboard_v3 {


const vnx::Hash64 RelayBoardV3Base::VNX_TYPE_HASH(0x59c86a8cb3f030daull);
const vnx::Hash64 RelayBoardV3Base::VNX_CODE_HASH(0xc3468ffb482f11aull);

RelayBoardV3Base::RelayBoardV3Base(const std::string& _vnx_name)
	:	Module::Module(_vnx_name)
{
	vnx::read_config(vnx_name + ".topics_board_to_ros", topics_board_to_ros);
	vnx::read_config(vnx_name + ".topics_ros_to_board", topics_ros_to_board);
	vnx::read_config(vnx_name + ".topics_from_board", topics_from_board);
	vnx::read_config(vnx_name + ".topics_to_ros", topics_to_ros);
	vnx::read_config(vnx_name + ".platform_interface_server", platform_interface_server);
	vnx::read_config(vnx_name + ".safety_server", safety_server);
	vnx::read_config(vnx_name + ".launcher_server", launcher_server);
	vnx::read_config(vnx_name + ".board_init_interval_ms", board_init_interval_ms);
	vnx::read_config(vnx_name + ".update_interval_ms", update_interval_ms);
	vnx::read_config(vnx_name + ".remote_config", remote_config);
	vnx::read_config(vnx_name + ".max_publish_queue_ros", max_publish_queue_ros);
	vnx::read_config(vnx_name + ".max_subscribe_queue_ros", max_subscribe_queue_ros);
	vnx::read_config(vnx_name + ".kinematic_type", kinematic_type);
}

vnx::Hash64 RelayBoardV3Base::get_type_hash() const {
	return VNX_TYPE_HASH;
}

std::string RelayBoardV3Base::get_type_name() const {
	return "neo_relayboard_v3.RelayBoardV3";
}

const vnx::TypeCode* RelayBoardV3Base::get_type_code() const {
	return neo_relayboard_v3::vnx_native_type_code_RelayBoardV3Base;
}

void RelayBoardV3Base::accept(vnx::Visitor& _visitor) const {
	const vnx::TypeCode* _type_code = neo_relayboard_v3::vnx_native_type_code_RelayBoardV3Base;
	_visitor.type_begin(*_type_code);
	_visitor.type_field(_type_code->fields[0], 0); vnx::accept(_visitor, topics_board_to_ros);
	_visitor.type_field(_type_code->fields[1], 1); vnx::accept(_visitor, topics_ros_to_board);
	_visitor.type_field(_type_code->fields[2], 2); vnx::accept(_visitor, topics_from_board);
	_visitor.type_field(_type_code->fields[3], 3); vnx::accept(_visitor, topics_to_ros);
	_visitor.type_field(_type_code->fields[4], 4); vnx::accept(_visitor, platform_interface_server);
	_visitor.type_field(_type_code->fields[5], 5); vnx::accept(_visitor, safety_server);
	_visitor.type_field(_type_code->fields[6], 6); vnx::accept(_visitor, launcher_server);
	_visitor.type_field(_type_code->fields[7], 7); vnx::accept(_visitor, board_init_interval_ms);
	_visitor.type_field(_type_code->fields[8], 8); vnx::accept(_visitor, update_interval_ms);
	_visitor.type_field(_type_code->fields[9], 9); vnx::accept(_visitor, remote_config);
	_visitor.type_field(_type_code->fields[10], 10); vnx::accept(_visitor, max_publish_queue_ros);
	_visitor.type_field(_type_code->fields[11], 11); vnx::accept(_visitor, max_subscribe_queue_ros);
	_visitor.type_field(_type_code->fields[12], 12); vnx::accept(_visitor, kinematic_type);
	_visitor.type_end(*_type_code);
}

void RelayBoardV3Base::write(std::ostream& _out) const {
	_out << "{";
	_out << "\"topics_board_to_ros\": "; vnx::write(_out, topics_board_to_ros);
	_out << ", \"topics_ros_to_board\": "; vnx::write(_out, topics_ros_to_board);
	_out << ", \"topics_from_board\": "; vnx::write(_out, topics_from_board);
	_out << ", \"topics_to_ros\": "; vnx::write(_out, topics_to_ros);
	_out << ", \"platform_interface_server\": "; vnx::write(_out, platform_interface_server);
	_out << ", \"safety_server\": "; vnx::write(_out, safety_server);
	_out << ", \"launcher_server\": "; vnx::write(_out, launcher_server);
	_out << ", \"board_init_interval_ms\": "; vnx::write(_out, board_init_interval_ms);
	_out << ", \"update_interval_ms\": "; vnx::write(_out, update_interval_ms);
	_out << ", \"remote_config\": "; vnx::write(_out, remote_config);
	_out << ", \"max_publish_queue_ros\": "; vnx::write(_out, max_publish_queue_ros);
	_out << ", \"max_subscribe_queue_ros\": "; vnx::write(_out, max_subscribe_queue_ros);
	_out << ", \"kinematic_type\": "; vnx::write(_out, kinematic_type);
	_out << "}";
}

void RelayBoardV3Base::read(std::istream& _in) {
	if(auto _json = vnx::read_json(_in)) {
		from_object(_json->to_object());
	}
}

vnx::Object RelayBoardV3Base::to_object() const {
	vnx::Object _object;
	_object["__type"] = "neo_relayboard_v3.RelayBoardV3";
	_object["topics_board_to_ros"] = topics_board_to_ros;
	_object["topics_ros_to_board"] = topics_ros_to_board;
	_object["topics_from_board"] = topics_from_board;
	_object["topics_to_ros"] = topics_to_ros;
	_object["platform_interface_server"] = platform_interface_server;
	_object["safety_server"] = safety_server;
	_object["launcher_server"] = launcher_server;
	_object["board_init_interval_ms"] = board_init_interval_ms;
	_object["update_interval_ms"] = update_interval_ms;
	_object["remote_config"] = remote_config;
	_object["max_publish_queue_ros"] = max_publish_queue_ros;
	_object["max_subscribe_queue_ros"] = max_subscribe_queue_ros;
	_object["kinematic_type"] = kinematic_type;
	return _object;
}

void RelayBoardV3Base::from_object(const vnx::Object& _object) {
	for(const auto& _entry : _object.field) {
		if(_entry.first == "board_init_interval_ms") {
			_entry.second.to(board_init_interval_ms);
		} else if(_entry.first == "kinematic_type") {
			_entry.second.to(kinematic_type);
		} else if(_entry.first == "launcher_server") {
			_entry.second.to(launcher_server);
		} else if(_entry.first == "max_publish_queue_ros") {
			_entry.second.to(max_publish_queue_ros);
		} else if(_entry.first == "max_subscribe_queue_ros") {
			_entry.second.to(max_subscribe_queue_ros);
		} else if(_entry.first == "platform_interface_server") {
			_entry.second.to(platform_interface_server);
		} else if(_entry.first == "remote_config") {
			_entry.second.to(remote_config);
		} else if(_entry.first == "safety_server") {
			_entry.second.to(safety_server);
		} else if(_entry.first == "topics_board_to_ros") {
			_entry.second.to(topics_board_to_ros);
		} else if(_entry.first == "topics_from_board") {
			_entry.second.to(topics_from_board);
		} else if(_entry.first == "topics_ros_to_board") {
			_entry.second.to(topics_ros_to_board);
		} else if(_entry.first == "topics_to_ros") {
			_entry.second.to(topics_to_ros);
		} else if(_entry.first == "update_interval_ms") {
			_entry.second.to(update_interval_ms);
		}
	}
}

vnx::Variant RelayBoardV3Base::get_field(const std::string& _name) const {
	if(_name == "topics_board_to_ros") {
		return vnx::Variant(topics_board_to_ros);
	}
	if(_name == "topics_ros_to_board") {
		return vnx::Variant(topics_ros_to_board);
	}
	if(_name == "topics_from_board") {
		return vnx::Variant(topics_from_board);
	}
	if(_name == "topics_to_ros") {
		return vnx::Variant(topics_to_ros);
	}
	if(_name == "platform_interface_server") {
		return vnx::Variant(platform_interface_server);
	}
	if(_name == "safety_server") {
		return vnx::Variant(safety_server);
	}
	if(_name == "launcher_server") {
		return vnx::Variant(launcher_server);
	}
	if(_name == "board_init_interval_ms") {
		return vnx::Variant(board_init_interval_ms);
	}
	if(_name == "update_interval_ms") {
		return vnx::Variant(update_interval_ms);
	}
	if(_name == "remote_config") {
		return vnx::Variant(remote_config);
	}
	if(_name == "max_publish_queue_ros") {
		return vnx::Variant(max_publish_queue_ros);
	}
	if(_name == "max_subscribe_queue_ros") {
		return vnx::Variant(max_subscribe_queue_ros);
	}
	if(_name == "kinematic_type") {
		return vnx::Variant(kinematic_type);
	}
	return vnx::Variant();
}

void RelayBoardV3Base::set_field(const std::string& _name, const vnx::Variant& _value) {
	if(_name == "topics_board_to_ros") {
		_value.to(topics_board_to_ros);
	} else if(_name == "topics_ros_to_board") {
		_value.to(topics_ros_to_board);
	} else if(_name == "topics_from_board") {
		_value.to(topics_from_board);
	} else if(_name == "topics_to_ros") {
		_value.to(topics_to_ros);
	} else if(_name == "platform_interface_server") {
		_value.to(platform_interface_server);
	} else if(_name == "safety_server") {
		_value.to(safety_server);
	} else if(_name == "launcher_server") {
		_value.to(launcher_server);
	} else if(_name == "board_init_interval_ms") {
		_value.to(board_init_interval_ms);
	} else if(_name == "update_interval_ms") {
		_value.to(update_interval_ms);
	} else if(_name == "remote_config") {
		_value.to(remote_config);
	} else if(_name == "max_publish_queue_ros") {
		_value.to(max_publish_queue_ros);
	} else if(_name == "max_subscribe_queue_ros") {
		_value.to(max_subscribe_queue_ros);
	} else if(_name == "kinematic_type") {
		_value.to(kinematic_type);
	} else {
		throw std::logic_error("no such field: '" + _name + "'");
	}
}

/// \private
std::ostream& operator<<(std::ostream& _out, const RelayBoardV3Base& _value) {
	_value.write(_out);
	return _out;
}

/// \private
std::istream& operator>>(std::istream& _in, RelayBoardV3Base& _value) {
	_value.read(_in);
	return _in;
}

const vnx::TypeCode* RelayBoardV3Base::static_get_type_code() {
	const vnx::TypeCode* type_code = vnx::get_type_code(VNX_TYPE_HASH);
	if(!type_code) {
		type_code = vnx::register_type_code(static_create_type_code());
	}
	return type_code;
}

std::shared_ptr<vnx::TypeCode> RelayBoardV3Base::static_create_type_code() {
	auto type_code = std::make_shared<vnx::TypeCode>();
	type_code->name = "neo_relayboard_v3.RelayBoardV3";
	type_code->type_hash = vnx::Hash64(0x59c86a8cb3f030daull);
	type_code->code_hash = vnx::Hash64(0xc3468ffb482f11aull);
	type_code->is_native = true;
	type_code->native_size = sizeof(::neo_relayboard_v3::RelayBoardV3Base);
	type_code->depends.resize(1);
	type_code->depends[0] = ::pilot::kinematic_type_e::static_get_type_code();
	type_code->methods.resize(9);
	type_code->methods[0] = ::vnx::ModuleInterface_vnx_get_config_object::static_get_type_code();
	type_code->methods[1] = ::vnx::ModuleInterface_vnx_get_config::static_get_type_code();
	type_code->methods[2] = ::vnx::ModuleInterface_vnx_set_config_object::static_get_type_code();
	type_code->methods[3] = ::vnx::ModuleInterface_vnx_set_config::static_get_type_code();
	type_code->methods[4] = ::vnx::ModuleInterface_vnx_get_type_code::static_get_type_code();
	type_code->methods[5] = ::vnx::ModuleInterface_vnx_get_module_info::static_get_type_code();
	type_code->methods[6] = ::vnx::ModuleInterface_vnx_restart::static_get_type_code();
	type_code->methods[7] = ::vnx::ModuleInterface_vnx_stop::static_get_type_code();
	type_code->methods[8] = ::vnx::ModuleInterface_vnx_self_test::static_get_type_code();
	type_code->fields.resize(13);
	{
		auto& field = type_code->fields[0];
		field.is_extended = true;
		field.name = "topics_board_to_ros";
		field.code = {13, 4, 12, 5, 32};
	}
	{
		auto& field = type_code->fields[1];
		field.is_extended = true;
		field.name = "topics_ros_to_board";
		field.code = {13, 3, 32, 13, 3, 32, 12, 5};
	}
	{
		auto& field = type_code->fields[2];
		field.is_extended = true;
		field.name = "topics_from_board";
		field.code = {12, 12, 5};
	}
	{
		auto& field = type_code->fields[3];
		field.is_extended = true;
		field.name = "topics_to_ros";
		field.code = {13, 3, 32, 32};
	}
	{
		auto& field = type_code->fields[4];
		field.is_extended = true;
		field.name = "platform_interface_server";
		field.value = vnx::to_string("PlatformInterface");
		field.code = {32};
	}
	{
		auto& field = type_code->fields[5];
		field.is_extended = true;
		field.name = "safety_server";
		field.value = vnx::to_string("SafetyInterface");
		field.code = {32};
	}
	{
		auto& field = type_code->fields[6];
		field.is_extended = true;
		field.name = "launcher_server";
		field.value = vnx::to_string("RelayBoardV3Launcher");
		field.code = {32};
	}
	{
		auto& field = type_code->fields[7];
		field.data_size = 4;
		field.name = "board_init_interval_ms";
		field.value = vnx::to_string(1500);
		field.code = {7};
	}
	{
		auto& field = type_code->fields[8];
		field.data_size = 4;
		field.name = "update_interval_ms";
		field.value = vnx::to_string(500);
		field.code = {7};
	}
	{
		auto& field = type_code->fields[9];
		field.is_extended = true;
		field.name = "remote_config";
		field.code = {24};
	}
	{
		auto& field = type_code->fields[10];
		field.data_size = 4;
		field.name = "max_publish_queue_ros";
		field.value = vnx::to_string(1);
		field.code = {7};
	}
	{
		auto& field = type_code->fields[11];
		field.data_size = 4;
		field.name = "max_subscribe_queue_ros";
		field.value = vnx::to_string(1);
		field.code = {7};
	}
	{
		auto& field = type_code->fields[12];
		field.is_extended = true;
		field.name = "kinematic_type";
		field.code = {19, 0};
	}
	type_code->build();
	return type_code;
}

void RelayBoardV3Base::vnx_handle_switch(std::shared_ptr<const vnx::Value> _value) {
	const auto* _type_code = _value->get_type_code();
	while(_type_code) {
		switch(_type_code->type_hash) {
			case 0xc6790e4d7b66f791ull:
				handle(std::static_pointer_cast<const ::pilot::BatteryState>(_value));
				return;
			case 0x77fc634da8371a8eull:
				handle(std::static_pointer_cast<const ::pilot::EmergencyState>(_value));
				return;
			case 0x1ca79bd1e6cc8028ull:
				handle(std::static_pointer_cast<const ::pilot::IOBoardData>(_value));
				return;
			case 0x80c07ca1b021de76ull:
				handle(std::static_pointer_cast<const ::pilot::Incident>(_value));
				return;
			case 0x83624e8e635643efull:
				handle(std::static_pointer_cast<const ::pilot::PowerState>(_value));
				return;
			case 0x9520c728da2f4f2full:
				handle(std::static_pointer_cast<const ::pilot::RelayBoardV3Data>(_value));
				return;
			case 0xfe0a52079ba87c3full:
				handle(std::static_pointer_cast<const ::pilot::SafetyState>(_value));
				return;
			case 0x6581fb0fdb31ddaeull:
				handle(std::static_pointer_cast<const ::pilot::SystemState>(_value));
				return;
			case 0x4850604e2930c0a0ull:
				handle(std::static_pointer_cast<const ::pilot::USBoardData>(_value));
				return;
			case 0x5c1d7427a54840d3ull:
				handle(std::static_pointer_cast<const ::pilot::kinematics::bicycle::DriveState>(_value));
				return;
			case 0x954b1e7cfbb6b85ull:
				handle(std::static_pointer_cast<const ::pilot::kinematics::differential::DriveState>(_value));
				return;
			case 0x746ce8edadd78a68ull:
				handle(std::static_pointer_cast<const ::pilot::kinematics::mecanum::DriveState>(_value));
				return;
			case 0x735822e6960c247ull:
				handle(std::static_pointer_cast<const ::pilot::kinematics::omnidrive::DriveState>(_value));
				return;
			case 0x2a13f6d072f9b852ull:
				handle(std::static_pointer_cast<const ::vnx::LogMsg>(_value));
				return;
			default:
				_type_code = _type_code->super;
		}
	}
	handle(std::static_pointer_cast<const vnx::Value>(_value));
}

std::shared_ptr<vnx::Value> RelayBoardV3Base::vnx_call_switch(std::shared_ptr<const vnx::Value> _method, const vnx::request_id_t& _request_id) {
	switch(_method->get_type_hash()) {
		case 0x17f58f68bf83abc0ull: {
			auto _args = std::static_pointer_cast<const ::vnx::ModuleInterface_vnx_get_config_object>(_method);
			auto _return_value = ::vnx::ModuleInterface_vnx_get_config_object_return::create();
			_return_value->_ret_0 = vnx_get_config_object();
			return _return_value;
		}
		case 0xbbc7f1a01044d294ull: {
			auto _args = std::static_pointer_cast<const ::vnx::ModuleInterface_vnx_get_config>(_method);
			auto _return_value = ::vnx::ModuleInterface_vnx_get_config_return::create();
			_return_value->_ret_0 = vnx_get_config(_args->name);
			return _return_value;
		}
		case 0xca30f814f17f322full: {
			auto _args = std::static_pointer_cast<const ::vnx::ModuleInterface_vnx_set_config_object>(_method);
			auto _return_value = ::vnx::ModuleInterface_vnx_set_config_object_return::create();
			vnx_set_config_object(_args->config);
			return _return_value;
		}
		case 0x362aac91373958b7ull: {
			auto _args = std::static_pointer_cast<const ::vnx::ModuleInterface_vnx_set_config>(_method);
			auto _return_value = ::vnx::ModuleInterface_vnx_set_config_return::create();
			vnx_set_config(_args->name, _args->value);
			return _return_value;
		}
		case 0x305ec4d628960e5dull: {
			auto _args = std::static_pointer_cast<const ::vnx::ModuleInterface_vnx_get_type_code>(_method);
			auto _return_value = ::vnx::ModuleInterface_vnx_get_type_code_return::create();
			_return_value->_ret_0 = vnx_get_type_code();
			return _return_value;
		}
		case 0xf6d82bdf66d034a1ull: {
			auto _args = std::static_pointer_cast<const ::vnx::ModuleInterface_vnx_get_module_info>(_method);
			auto _return_value = ::vnx::ModuleInterface_vnx_get_module_info_return::create();
			_return_value->_ret_0 = vnx_get_module_info();
			return _return_value;
		}
		case 0x9e95dc280cecca1bull: {
			auto _args = std::static_pointer_cast<const ::vnx::ModuleInterface_vnx_restart>(_method);
			auto _return_value = ::vnx::ModuleInterface_vnx_restart_return::create();
			vnx_restart();
			return _return_value;
		}
		case 0x7ab49ce3d1bfc0d2ull: {
			auto _args = std::static_pointer_cast<const ::vnx::ModuleInterface_vnx_stop>(_method);
			auto _return_value = ::vnx::ModuleInterface_vnx_stop_return::create();
			vnx_stop();
			return _return_value;
		}
		case 0x6ce3775b41a42697ull: {
			auto _args = std::static_pointer_cast<const ::vnx::ModuleInterface_vnx_self_test>(_method);
			auto _return_value = ::vnx::ModuleInterface_vnx_self_test_return::create();
			_return_value->_ret_0 = vnx_self_test();
			return _return_value;
		}
	}
	auto _ex = vnx::NoSuchMethod::create();
	_ex->dst_mac = vnx_request ? vnx_request->dst_mac : vnx::Hash64();
	_ex->method = _method->get_type_name();
	return _ex;
}


} // namespace neo_relayboard_v3


namespace vnx {

void read(TypeInput& in, ::neo_relayboard_v3::RelayBoardV3Base& value, const TypeCode* type_code, const uint16_t* code) {
	if(code) {
		switch(code[0]) {
			case CODE_OBJECT:
			case CODE_ALT_OBJECT: {
				Object tmp;
				vnx::read(in, tmp, type_code, code);
				value.from_object(tmp);
				return;
			}
			case CODE_DYNAMIC:
			case CODE_ALT_DYNAMIC:
				vnx::read_dynamic(in, value);
				return;
		}
	}
	if(!type_code) {
		vnx::skip(in, type_code, code);
		return;
	}
	if(code) {
		switch(code[0]) {
			case CODE_STRUCT: type_code = type_code->depends[code[1]]; break;
			case CODE_ALT_STRUCT: type_code = type_code->depends[vnx::flip_bytes(code[1])]; break;
			default: {
				vnx::skip(in, type_code, code);
				return;
			}
		}
	}
	const char* const _buf = in.read(type_code->total_field_size);
	if(type_code->is_matched) {
		if(const auto* const _field = type_code->field_map[7]) {
			vnx::read_value(_buf + _field->offset, value.board_init_interval_ms, _field->code.data());
		}
		if(const auto* const _field = type_code->field_map[8]) {
			vnx::read_value(_buf + _field->offset, value.update_interval_ms, _field->code.data());
		}
		if(const auto* const _field = type_code->field_map[10]) {
			vnx::read_value(_buf + _field->offset, value.max_publish_queue_ros, _field->code.data());
		}
		if(const auto* const _field = type_code->field_map[11]) {
			vnx::read_value(_buf + _field->offset, value.max_subscribe_queue_ros, _field->code.data());
		}
	}
	for(const auto* _field : type_code->ext_fields) {
		switch(_field->native_index) {
			case 0: vnx::read(in, value.topics_board_to_ros, type_code, _field->code.data()); break;
			case 1: vnx::read(in, value.topics_ros_to_board, type_code, _field->code.data()); break;
			case 2: vnx::read(in, value.topics_from_board, type_code, _field->code.data()); break;
			case 3: vnx::read(in, value.topics_to_ros, type_code, _field->code.data()); break;
			case 4: vnx::read(in, value.platform_interface_server, type_code, _field->code.data()); break;
			case 5: vnx::read(in, value.safety_server, type_code, _field->code.data()); break;
			case 6: vnx::read(in, value.launcher_server, type_code, _field->code.data()); break;
			case 9: vnx::read(in, value.remote_config, type_code, _field->code.data()); break;
			case 12: vnx::read(in, value.kinematic_type, type_code, _field->code.data()); break;
			default: vnx::skip(in, type_code, _field->code.data());
		}
	}
}

void write(TypeOutput& out, const ::neo_relayboard_v3::RelayBoardV3Base& value, const TypeCode* type_code, const uint16_t* code) {
	if(code && code[0] == CODE_OBJECT) {
		vnx::write(out, value.to_object(), nullptr, code);
		return;
	}
	if(!type_code || (code && code[0] == CODE_ANY)) {
		type_code = neo_relayboard_v3::vnx_native_type_code_RelayBoardV3Base;
		out.write_type_code(type_code);
		vnx::write_class_header<::neo_relayboard_v3::RelayBoardV3Base>(out);
	}
	else if(code && code[0] == CODE_STRUCT) {
		type_code = type_code->depends[code[1]];
	}
	char* const _buf = out.write(16);
	vnx::write_value(_buf + 0, value.board_init_interval_ms);
	vnx::write_value(_buf + 4, value.update_interval_ms);
	vnx::write_value(_buf + 8, value.max_publish_queue_ros);
	vnx::write_value(_buf + 12, value.max_subscribe_queue_ros);
	vnx::write(out, value.topics_board_to_ros, type_code, type_code->fields[0].code.data());
	vnx::write(out, value.topics_ros_to_board, type_code, type_code->fields[1].code.data());
	vnx::write(out, value.topics_from_board, type_code, type_code->fields[2].code.data());
	vnx::write(out, value.topics_to_ros, type_code, type_code->fields[3].code.data());
	vnx::write(out, value.platform_interface_server, type_code, type_code->fields[4].code.data());
	vnx::write(out, value.safety_server, type_code, type_code->fields[5].code.data());
	vnx::write(out, value.launcher_server, type_code, type_code->fields[6].code.data());
	vnx::write(out, value.remote_config, type_code, type_code->fields[9].code.data());
	vnx::write(out, value.kinematic_type, type_code, type_code->fields[12].code.data());
}

void read(std::istream& in, ::neo_relayboard_v3::RelayBoardV3Base& value) {
	value.read(in);
}

void write(std::ostream& out, const ::neo_relayboard_v3::RelayBoardV3Base& value) {
	value.write(out);
}

void accept(Visitor& visitor, const ::neo_relayboard_v3::RelayBoardV3Base& value) {
	value.accept(visitor);
}

} // vnx
