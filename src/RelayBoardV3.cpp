/*
 * RelayBoardV3.cpp
 *
 *  Created on: Jan 19, 2022
 *      Author: jaw
 */


#include <pilot/relayboardv3/RelayBoardV3.h>


namespace pilot{
namespace relayboardv3{


RelayBoardV3::RelayBoardV3(const std::string &_vnx_name):
	RelayBoardV3Base(_vnx_name)
{
}


void RelayBoardV3::init(){
}


void RelayBoardV3::main(){
	Super::main();
}


} // relayboardv3
} // pilot



