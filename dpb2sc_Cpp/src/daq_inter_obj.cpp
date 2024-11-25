#ifdef DAQ_MODE
    #include <daqinterface/DAQInterface.h>
    #include <string.h>
	std::string interface_config_file = "/home/petalinux/daq/InterfaceConfig";
	ToolFramework::DAQInterface DAQ_Inter(interface_config_file);
#endif