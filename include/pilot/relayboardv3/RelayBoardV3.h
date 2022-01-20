/*
 * RelayBoardV3.h
 *
 *  Created on: Jan 19, 2022
 *      Author: jaw
 */

#ifndef INCLUDE_pilot_relayboardv3_RelayBoardV3_H
#define INCLUDE_pilot_relayboardv3_RelayBoardV3_H

#include <pilot/relayboardv3/RelayBoardV3Base.hxx>


namespace pilot{
namespace relayboardv3{


class RelayBoardV3 : public RelayBoardV3Base{
public:
	RelayBoardV3(const std::string &_vnx_name);

protected:
	void init() override;

	void main() override;
};


} // relayboardv3
} // pilot


#endif /* INCLUDE_pilot_relayboardv3_RelayBoardV3_H */

