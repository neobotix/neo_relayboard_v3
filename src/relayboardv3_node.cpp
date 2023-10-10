#include <vnx/vnx.h>
#include <vnx/Proxy.h>
#include <vnx/Terminal.h>

#include <neo_relayboard_v3/RelayBoardV3.h>

#include <rclcpp/rclcpp.hpp>


int main(int argc, char **argv){
	const std::string process_name = "relayboardv3_node";

	// initialize ROS
	rclcpp::init(argc, argv);

	// initialize VNX
	vnx::init(process_name, 0, nullptr);

	std::shared_ptr<rclcpp::Node> nh = std::make_shared<rclcpp::Node>(process_name);
	nh->declare_parameter<std::string>("board_node", "192.168.1.80:5554");
	nh->declare_parameter<std::string>("pilot_config", "config/default/generic/");

	std::string board_node;
	std::string pilot_config;
	nh->get_parameter<std::string>("board_node", board_node);
	nh->get_parameter<std::string>("pilot_config", pilot_config);

	vnx::read_config_tree(pilot_config);

	{
		vnx::Handle<vnx::Terminal> module = new vnx::Terminal("Terminal");
		module.start_detached();
	}

	{
		vnx::Handle<neo_relayboard_v3::RelayBoardV3> module = new neo_relayboard_v3::RelayBoardV3("RelayBoardV3Node", nh);
		{
			vnx::Handle<vnx::Proxy> proxy = new vnx::Proxy("RelayBoardV3_Proxy", vnx::Endpoint::from_url(board_node));
			for(const auto &entry : module->topics_board_to_ros){
				proxy->import_list.push_back(entry.first);
			}
			for(const auto &topic : module->topics_from_board){
				proxy->import_list.push_back(topic);
			}
			for(const auto &entry : module->topics_ros_to_board){
				for(const auto &types : entry.second){
					proxy->export_list.push_back(types.second);
				}
			}
			proxy->forward_list.push_back(module->platform_interface_server);
			proxy->forward_list.push_back(module->safety_server);
			proxy->forward_list.push_back(module->launcher_server);
			proxy.start_detached();
		}
		module.start_detached();
	}

	rclcpp::spin(nh);
	vnx::close();

	return 0;
}



