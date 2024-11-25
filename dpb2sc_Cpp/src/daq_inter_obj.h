#ifndef __DAQ_INTER_OBJ_H_INCLUDED__
#define __DAQ_INTER_OBJ_H_INCLUDED__

#ifdef DAQ_MODE
    #include <daqinterface/DAQInterface.h>
	extern ToolFramework::DAQInterface DAQ_Inter;
#endif

#endif