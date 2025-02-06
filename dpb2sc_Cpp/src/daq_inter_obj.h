#ifndef __DAQ_INTER_OBJ_H_INCLUDED__
#define __DAQ_INTER_OBJ_H_INCLUDED__

#ifdef DAQ_MODE
    #include <daqinterface/DAQInterface.h>

    #define ENV_PARAM 0
    #define CHAN_PARAM 1
	#define VARIABLE_TYPE 0
	#define OPTIONS_TYPE 1
	#define BUTTONS_TYPE 2

	extern ToolFramework::DAQInterface* DAQ_Inter;
    struct slow_control_var_struct  {
        int type;
        int chan_or_env;
        int chan_n;
        char name[64];
        int min;
        int max;
        int step;
        int default_value;
        std::string options[2];
    };

    extern slow_control_var_struct DAQ_chan_cmd_list[70];
#endif

#endif