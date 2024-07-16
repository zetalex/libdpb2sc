//
//  COPacketCmdHkTdm.h
//
//  Created by Faber on 13 Jul 2022
//  Copyright © 2018 Faber. All rights reserved.
//
#ifndef COPacketCmdHkDig_h
#define COPacketCmdHkDig_h

#include <stdint.h>
#include "common/protocols/COPacket/COPacket.hpp"

// This enum defines the numeric values of command codes
enum HKDIG_CMD_LIST {
	HKDIG_GET_THR_NUM = 0,       // Set channel threshold
	HKDIG_SET_THR_NUM,           // Get channel threshold
	HKDIG_SET_THR_ALL,           // Set same threshold for all channels

	HKDIG_GET_IT_NUM,
	HKDIG_SET_IT_NUM,
	HKDIG_SET_IT_ALL,

	HKDIG_GET_DT_NUM,
	HKDIG_SET_DT_NUM,
	HKDIG_SET_DT_ALL,

	HKDIG_EN_CAL_N,				// Enable channel calibration input
	HKDIG_DIS_CAL_N,			// Disable channel calibration input

	// Channel commands
	HKDIG_GET_CHN_STATUS,			// Get Channel Status Reg
	HKDIG_GET_CHN_CNTRL,			// Get Channel Control Reg
	HKDIG_SET_CHN_CNTRL,			// Set Channel Status Reg
	HKDIG_START_FE_DAQ,
	HKDIG_START_DAQ,
	HKDIG_SET_PED_TYPE,
	HKDIG_TDC_RST,

	// Board Commands
	HKDIG_GET_GW_VER,
	HKDIG_GET_SW_VER,
	HKDIG_GET_BOARD_STATUS,
	HKDIG_GET_BOARD_CNTRL,
	HKDIG_GET_UPTIME,

	// Rate Monitor commands
	HKDIG_GET_RMON_T,
	HKDIG_SET_RMON_T,
	HKDIG_GET_RMON_N,		// Get channel RMon
	HKDIG_RUN_RMON,

	// Monitoring
    HKDIG_GET_BOARD_3V3A,
    HKDIG_GET_BOARD_12VA,
    HKDIG_GET_BOARD_I12V,
    HKDIG_GET_BOARD_5V0A,
    HKDIG_GET_BOARD_5V0F,
    HKDIG_GET_BOARD_C12V,
    HKDIG_GET_BOARD_I5VF,
    HKDIG_GET_BOARD_I3V3A,
    HKDIG_GET_BOARD_I12VA,
    HKDIG_GET_BOARD_TU40,
    HKDIG_GET_BOARD_TU41,
    HKDIG_GET_BOARD_TU45,

    HKDIG_HELP,
    HKDIG_ERRO
};

#endif /* COPacketCmdHkDig_h */
