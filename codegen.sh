#!/bin/bash

VNX_INTERFACE_DIR=${VNX_INTERFACE_DIR:-/usr/interface}

VNX_INCLUDE=$1

if [ -z "$VNX_INCLUDE" ]; then
	VNX_INCLUDE=${VNX_INTERFACE_DIR}/vnx/
fi

PILOT_INCLUDE="pilot-public/interface/ pilot-public/kinematics/interface/ pilot-public/kinematics/differential/interface/ pilot-public/kinematics/mecanum/interface/ pilot-public/kinematics/omnidrive/interface/"

cd $(dirname "$0")

./pilot-public/codegen.sh "$VNX_INCLUDE"
vnxcppcodegen --cleanup generated/ neo_relayboard_v3 modules/ $VNX_INCLUDE $PILOT_INCLUDE


