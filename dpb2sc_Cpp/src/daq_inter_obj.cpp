#ifdef DAQ_MODE
    #include <daqinterface/DAQInterface.h>
    #include <string.h>
	std::string interface_config_file = "/home/petalinux/daq/InterfaceConfig";
	ToolFramework::DAQInterface DAQ_Inter(interface_config_file);

        const char *LV_chan_cmd_list[3] = {
        "SET_LV_STATUS",
        "SET_LV_VOLT",
        "SET_LV_CURR"
    };
    const char *HV_chan_cmd_list[6] = {
        "SET_HV_STATUS",
        "SET_HV_VOLT",
        "SET_HV_CURR",
        "SET_HV_RAMPUP",
        "SET_HV_RAMPDOWN",
        "SET_HV_TRIP"
    };
    // Remember the CPU Setting of LV and HV

    const char *DIG0_chan_cmd_list[8] = {
        "SET_DIG0_STATUS",
        "SET_DIG0_DISCTRES"
        "SET_DIG0_INTTIME",
        "SET_DIG0_CALIB",
        "SET_DIG0_FESTATUS",
        "SET_DIG0_DAQSTATUS",
        "SET_DIG0_PEDTYPE",
        "SET_DIG0_DEADTIME"
    };
    const char *DIG0_env_cmd_list[13] = {
        "SET_DIG0_CALIBPULSE",
        "SET_DIG0_CALIBPWR"
        "SET_DIG0_CALIBLEN",
        "SET_DIG0_CALIBAMP",
        "SET_DIG0_CALIBPDN",
        "SET_DIG0_CALIBMUTE",
        "SET_DIG0_CALIBSEN",
        "SET_DIG0_ODSEL",
        "SET_DIG0_TDCRST",
        "SET_DIG0_AURORARST",
        "SET_DIG0_CLOCK",
        "SET_DIG0_RMONT",
        "SET_DIG0_CLOCK"
    };

    const char *DIG1_chan_cmd_list[8] = {
        "SET_DIG1_STATUS",
        "SET_DIG1_DISCTRES"
        "SET_DIG1_INTTIME",
        "SET_DIG1_CALIB",
        "SET_DIG1_FESTATUS",
        "SET_DIG1_DAQSTATUS",
        "SET_DIG1_PEDTYPE",
        "SET_DIG1_DEADTIME"
    };
    const char *DIG1_env_cmd_list[13] = {
        "SET_DIG1_CALIBPULSE",
        "SET_DIG1_CALIBPWR"
        "SET_DIG1_CALIBLEN",
        "SET_DIG1_CALIBAMP",
        "SET_DIG1_CALIBPDN",
        "SET_DIG1_CALIBMUTE",
        "SET_DIG1_CALIBSEN",
        "SET_DIG1_ODSEL",
        "SET_DIG1_TDCRST",
        "SET_DIG1_AURORARST",
        "SET_DIG1_CLOCK",
        "SET_DIG1_RMONT",
        "SET_DIG1_CLOCK"
    };

    const char *DPB_sfp_cmd_list[3] {
        "SET_DPB_STATUS",
        "SET_DPB_TEMP",
        "SET_DPB_CURR"
    };

    const char *DPB_env_cmd_list[11] {
        "SET_DPB_STATUS_ETH0",
        "SET_DPB_STATUS_ETH1",
        "SET_DPB_TEMP_PCB",
        "SET_DPB_TEMP_FPGA",
        "SET_DPB_TEMP_FPDCPU",
        "SET_DPB_TEMP_LPDCPU",
        "SET_DPB_VOLT_FPDCPU",
        "SET_DPB_VOLT_LPDCPU",
        "SET_DPB_CURR_12V",
        "SET_DPB_CURR_3V3",
        "SET_DPB_CURR_1V8"
    };
#endif