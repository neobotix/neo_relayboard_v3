#include <vnx/vnx.h>
#include <vnx/Terminal.h>

#include <pilot/relayboardv3/RelayBoardV3.h>


int main(int argc, char **argv){
	std::map<std::string, std::string> options;
	vnx::init("relayboardv3_node", argc, argv, options);

	{
		vnx::Handle<vnx::Terminal> module = new vnx::Terminal("Terminal");
		module.start_detached();
	}
	{
		vnx::Handle<pilot::relayboardv3::RelayBoardV3> module = new pilot::relayboardv3::RelayBoardV3("RelayBoardV3");
		module.start_detached();
	}

	vnx::wait();
	vnx::close();

	return 0;
}



