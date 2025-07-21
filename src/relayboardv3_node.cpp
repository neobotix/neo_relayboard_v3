#include <vnx/vnx.h>
#include <vnx/Server.h>
#include <vnx/Proxy.h>
#include <vnx/Terminal.h>

#include <neo_relayboard_v3/RelayBoardV3.h>

#include <ros/ros.h>

int main(int argc, char **argv){
    const std::string process_name = "relayboardv3_node";

    // initialize ROS
    ros::init(argc, argv, process_name);
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // initialize VNX
    vnx::init(process_name, 0, nullptr);

    std::string board_node;
    std::string pilot_config;
    std::string agm_batteries_config;
    std::string lfp_batteries_config;
    std::string multipowr_config;
    bool lfp_batteries = false;
    bool use_multipowr = false;

    nh.param<std::string>("board_node", board_node, "192.168.1.80:5554");
    nh.param<std::string>("pilot_config", pilot_config, "config/default/generic/");
    nh.param<bool>("lfp_batteries", lfp_batteries, false);
    nh.param<bool>("use_multipowr", use_multipowr, false);
    nh.param<std::string>("multipowr_config", multipowr_config, "config/addons/smart-charger/");
    nh.param<std::string>("agm_batteries_config", agm_batteries_config, "config/addons/agm-batteries/");
    nh.param<std::string>("lfp_batteries_config", lfp_batteries_config, "config/addons/lfp-batteries/");

    vnx::read_config_tree(pilot_config);

    // Read LFP battery configs if available
    if (lfp_batteries) {
        vnx::read_config_tree(lfp_batteries_config);
    } else {
        vnx::read_config_tree(agm_batteries_config);
    }

    // Read smart charger configs if available
    if (use_multipowr) {
        vnx::read_config_tree(multipowr_config);
    }

    {
        // disable vnx log, since the messages will be printed by ROS
        vnx::ProcessClient process(process_name);
        process.pause_log();
    }

    {
        vnx::Handle<vnx::Server> module = new vnx::Server("TcpServer", vnx::Endpoint::from_url("0.0.0.0:5555"));
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

    ros::spin();
    vnx::close();

    return 0;
}



