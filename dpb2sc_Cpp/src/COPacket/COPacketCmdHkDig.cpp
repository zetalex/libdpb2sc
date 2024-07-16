//
//  COPacketCmdHkDig.cpp
//
//  Created by Faber on 13 Jul 2022
//  Copyright © 2018 Faber. All rights reserved.
//

#include "COPacketCmdHkDig.h"

const uint16_t   HkDigCmdSize = 41;
_COPacketCmdType HkDigCommands[HkDigCmdSize] = {
	{"gthn", HKDIG_GET_THR_NUM,      " ch# Get ch thr"},
	{"sthn", HKDIG_SET_THR_NUM,      " ch thr# Set ch thr"},
	{"stha", HKDIG_SET_THR_ALL,      " thr# Set all thr"},

	{"gitn", HKDIG_GET_IT_NUM,       " ch# Get ch intT"},
	{"sitn", HKDIG_SET_IT_NUM,       " ch it# Set ch intT"},
	{"sita", HKDIG_SET_IT_ALL,       " it# Set all intT"},

	{"gdtn", HKDIG_GET_DT_NUM,       " ch# Get ch deadT"},
	{"sdtn", HKDIG_SET_DT_NUM,       " ch dt# Set ch deadT"},
	{"sdta", HKDIG_SET_DT_ALL,       " dt# Set all deadT"},

	{"ecaln", HKDIG_EN_CAL_N,        " ch# En ch cal"},
	{"dcaln", HKDIG_DIS_CAL_N,       " ch# Dis ch cal"},

	{"gchs", HKDIG_GET_CHN_STATUS,   " ch# Get ch status"},
	{"gchc", HKDIG_GET_CHN_CNTRL,    " ch# Get ch control"},
	{"schc", HKDIG_SET_CHN_CNTRL,    " ch# Set ch control"},
	{"efen", HKDIG_START_FE_DAQ,     " ch# En ch FE"},
	{"edqn", HKDIG_START_DAQ,        " ch# En ch DAQ"},
	{"sped", HKDIG_SET_PED_TYPE,     " ch ped_type# Set ch ped type"},
	{"tdcr", HKDIG_TDC_RST,          "# TdcC Rst"},

	{"ggwv", HKDIG_GET_GW_VER,       "# Get GW Ver"},
	{"gswv", HKDIG_GET_SW_VER,       "# Get SW Ver"},
	{"gbds", HKDIG_GET_BOARD_STATUS, "# Get brd status"},
	{"gbdc", HKDIG_GET_BOARD_CNTRL,  "# Get brd control"},
	{"gupt", HKDIG_GET_UPTIME,       "# Get uptime"},

	{"grmt", HKDIG_GET_RMON_T,       "# Get RMon T"},
	{"srmt", HKDIG_SET_RMON_T,       "# Set RMon T"},
	{"grmv", HKDIG_GET_RMON_N,       " ch# Get ch RMon"},
	{"runr", HKDIG_RUN_RMON,         "# Run RMon"},

	{"g3v3a", HKDIG_GET_BOARD_3V3A,   "# Get 3V3A"},
	{"g12va", HKDIG_GET_BOARD_12VA,   "# Get 12VA"},
	{"gi12v", HKDIG_GET_BOARD_I12V,   "# Get I12V"},
	{"g5v0a", HKDIG_GET_BOARD_5V0A,   "# Get 5V0A"},
	{"g5v0f", HKDIG_GET_BOARD_5V0F,   "# Get 5V0F"},
	{"gc12v", HKDIG_GET_BOARD_C12V,   "# Get C12V"},
	{"gi5vf",  HKDIG_GET_BOARD_I5VF,  "# Get I5VF"},
	{"gi3v3a", HKDIG_GET_BOARD_I3V3A, "# Get I3V3A"},
	{"gi12va", HKDIG_GET_BOARD_I12VA, "# Get I12VA"},
	{"gtu40",  HKDIG_GET_BOARD_TU40,  "# Get TU40"},
	{"gtu41",  HKDIG_GET_BOARD_TU41,  "# Get TU41"},
	{"gtu45",  HKDIG_GET_BOARD_TU45,  "# Get TU45"},

    {"help", HKDIG_HELP,             "# Help"},
    {"erro", HKDIG_ERRO,             "#"}
};

_COPacketCmdList HkDigCmdList = {
    HkDigCmdSize,
    HkDigCommands
};
