#include <vnx/vnx.h>
#include <vnx/Proxy.h>
#include <vnx/Terminal.h>

#include <pilot/relayboardv3/RelayBoardV3.h>

#include <ros/ros.h>


int main(int argc, char **argv){
	const std::string program_name = "relayboardv3_node";
	// initialize ROS
	ros::init(argc, argv, program_name);

	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	// initialize VNX
	vnx::init(program_name, 0, nullptr);

	std::string board_node;
	std::string pilot_config;
	nh_private.param<std::string>("board_node", board_node, "192.168.1.80:5554");
	nh_private.param<std::string>("pilot_config", pilot_config, "config/default/generic/");
	vnx::read_config_tree(pilot_config);

	{
		vnx::Handle<vnx::Terminal> module = new vnx::Terminal("Terminal");
		module.start_detached();
	}
	{
		vnx::Handle<pilot::relayboardv3::RelayBoardV3> module = new pilot::relayboardv3::RelayBoardV3("RelayBoardV3");
		module.start_detached();
		{
			vnx::Handle<vnx::Proxy> proxy = new vnx::Proxy("Proxy", vnx::Endpoint::from_url(board_node));
			proxy->time_sync = true;
			for(const auto &entry : module->topics_board_to_ros){
				proxy->import_list.push_back(entry.first);
			}
			proxy.start_detached();
		}
	}

	ros::spin();
	vnx::close();

	return 0;
}



