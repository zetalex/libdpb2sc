/************************** Libraries includes *****************************/
// COPacket includes
#include <common/protocols/COPacket/COPacket.hpp>

extern _COPacketCmdList HkDigCmdList;

extern "C" {
#include "dpb2sc.h"

/************************** Init and Close functions ******************************/
/** @defgroup init_close Init and close functions
 *  Functions to initialize and close different elements required by the library. lib_init must be called before using any of the other functions
 *  @{
 */

int dpbsc_lib_init(struct DPB_I2cSensors *data) {

	int rc = 0;
	int var_lock;

	rc = init_semaphores();
	if(rc)
		return rc;
	get_GPIO_base_address(&GPIO_BASE_ADDRESS);
	rc = zmq_socket_init(); //Initialize ZMQ Sockets
	if (rc) {
		printf("Error\r\n");
		return rc;
	}
	rc = init_I2cSensors(data); //Initialize i2c sensors
	if (rc) {
		printf("Error\r\n");
		return rc;
	}
	rc = init_shared_memory();
	if (rc) {
		printf("Error\r\n");
		return rc;
	}

	// try to create lock file in /var/lock
	var_lock = open("/var/lock/LCK..ttyUL1", O_CREAT | O_WRONLY | O_TRUNC | O_EXCL, 0644);
	if(var_lock < 0){
		printf("Cannot lock file ttyUL1. Already in use by other process");
		return errno;
	}
	write(var_lock, "%4d\n", getpid());
	close(var_lock);
	var_lock = open("/var/lock/LCK..ttyUL2", O_CREAT | O_WRONLY | O_TRUNC | O_EXCL, 0644);
	if(var_lock < 0){
		printf("Cannot lock file ttyUL2. Already in use by other process");
		return errno;
	}
	write(var_lock, "%4d\n", getpid());
	close(var_lock);
	// try to create lock file in /var/lock
	var_lock = open("/var/lock/LCK..ttyUL3", O_CREAT | O_WRONLY | O_TRUNC | O_EXCL, 0644);
	if(var_lock < 0){
		printf("Cannot lock file ttyUL3. Already in use by other process");
		return errno;
	}
	write(var_lock, "%4d\n", getpid());
	close(var_lock);
	var_lock = open("/var/lock/LCK..ttyUL4", O_CREAT | O_WRONLY | O_TRUNC | O_EXCL, 0644);
	if(var_lock < 0){
		printf("Cannot lock file ttyUL4. Already in use by other process");
		return errno;
	}
	write(var_lock, "%4d\n", getpid());
	close(var_lock);

	// Populate all electronics hash tables
	populate_lv_hash_table(LV_CMD_TABLE_SIZE,lv_daq_words,lv_board_words);
	populate_hv_hash_table(HV_CMD_TABLE_SIZE,hv_daq_words,hv_board_words);
	populate_dig_hash_table(DIG_STANDARD_CMD_TABLE_SIZE, dig_dpb_words);

	// Enable HV LV driver
	write_GPIO(HVLV_DRV_ENABLE_GPIO_OFFSET,1);
	//Enable Main CPUs of both HV and LV
	write_GPIO(LV_MAIN_CPU_GPIO_OFFSET,1);
	write_GPIO(HV_MAIN_CPU_GPIO_OFFSET,1);

	// Check if Dig0 and Dig1 are there
	char buffer[40];
	int serial_port_fd,n;
	CCOPacket pkt(COPKT_DEFAULT_START, COPKT_DEFAULT_STOP, COPKT_DEFAULT_SEP);

	serial_port_fd = open("/dev/ttyUL1",O_RDWR);
	setup_serial_port(serial_port_fd);
	pkt.CreatePacket(buffer, HkDigCmdList.CmdList[HKDIG_GET_GW_VER].CmdString);
	write(serial_port_fd, buffer, strlen(buffer));
	usleep(1000000);
	n = read(serial_port_fd, buffer, sizeof(buffer));
	if(n){
		pkt.LoadString(buffer);
		int16_t cmd_id = pkt.GetNextFiedlAsCOMMAND(HkDigCmdList);
		uint16_t gw_ver;
		pkt.GetNextFieldAsUINT16(gw_ver);
		sprintf(DIG0_SN,"%d",gw_ver);
		printf("Digitizer 0 has been detected: GW Version %s \n",DIG0_SN);
		dig0_connected = 1;
	}
	close(serial_port_fd);
	
	serial_port_fd = open("/dev/ttyUL2",O_RDWR);
	setup_serial_port(serial_port_fd);
	pkt.CreatePacket(buffer, HkDigCmdList.CmdList[HKDIG_GET_GW_VER].CmdString);
	write(serial_port_fd, buffer, strlen(buffer));
	usleep(1000000);
	n = read(serial_port_fd, buffer, sizeof(buffer));
	if(n){
		pkt.LoadString(buffer);
		int16_t cmd_id = pkt.GetNextFiedlAsCOMMAND(HkDigCmdList);
		if(cmd_id == HKDIG_GET_GW_VER){	
			uint16_t gw_ver;
			pkt.GetNextFieldAsUINT16(gw_ver);
			sprintf(DIG1_SN,"%d",gw_ver);
			printf("Digitizer 1 has been detected: GW Version %s \n",DIG1_SN);
			dig1_connected = 1;
		}
	}
	close(serial_port_fd);

	// Check if HV and LV are there
	serial_port_fd = open("/dev/ttyUL3",O_RDWR);
	usleep(1000000);
	setup_serial_port(serial_port_fd);
	write(serial_port_fd, "$BD:1,$CMD:MON,PAR:BDSNUM\r\n", strlen("$BD:1,$CMD:MON,PAR:BDSNUM\r\n"));
	usleep(1000000);
	n = read(serial_port_fd, buffer, sizeof(buffer));
	buffer[n] = '\0';
	if(n){
		for(int i = 12; i <=17; i++ ){ // Take just serial number from the response
			HV_SN[i-12] = buffer[i];
		}
		printf("HV has been detected: S/N %s \n",HV_SN);
		hv_connected = 1;
	}
	write(serial_port_fd, "$BD:0,$CMD:MON,PAR:BDSNUM\r\n", strlen("$BD:0,$CMD:MON,PAR:BDSNUM\r\n"));
	usleep(1000000);
	n = read(serial_port_fd, buffer, sizeof(buffer));
	buffer[n] = '\0';
	if(n){
		for(int i = 12; i <=17; i++ ){ // Take just serial number from the response
			LV_SN[i-12] = buffer[i];
		}
		printf("LV has been detected: S/N %s \n", LV_SN);
		lv_connected = 1;
	}
	close(serial_port_fd);

	char digcmd[32];
	char dig_response[64];
	int32_t commands[3]= {HKDIG_GET_BME_TCAL,HKDIG_GET_BME_HCAL,HKDIG_GET_BME_PCAL};
	char *temp;
	// Get Calibration variables from digitizers. They are read only variables written by the BME280 manufacturer
	if(dig0_connected){
		for(int i = 0; i < 3; i++){
			pkt.CreatePacket(digcmd, HkDigCmdList.CmdList[commands[i]].CmdString);
			dig_command_handling(0, digcmd, dig_response);
			pkt.LoadString(dig_response);
			printf("%s\n",dig_response);
			int32_t cmdIdx = pkt.GetNextFiedlAsCOMMAND(HkDigCmdList);
			switch(i){
				case 0:
				temp = pkt.GetNextField();
				strcpy(dig0_calT,temp);
				break;
				case 1:
				temp = pkt.GetNextField();
				strcpy(dig0_calH,temp);
				break;
				case 2:
				temp = pkt.GetNextField();
				strcpy(dig0_calP,temp);
				break;
			}
		}
	}

	if(dig1_connected){
		for(int i = 0; i < 3; i++){
			pkt.CreatePacket(digcmd, HkDigCmdList.CmdList[commands[i]].CmdString);
			dig_command_handling(1, digcmd, dig_response);
			pkt.LoadString(dig_response);
			int32_t cmdIdx = pkt.GetNextFiedlAsCOMMAND(HkDigCmdList);
			switch(i){
				case 0:
				temp = pkt.GetNextField();
				strcpy(dig1_calT,temp);
				break;
				case 1:
				temp = pkt.GetNextField();
				strcpy(dig1_calH,temp);
				break;
				case 2:
				temp = pkt.GetNextField();
				strcpy(dig1_calP,temp);
				break;
			}
		}
	}
	return 0;
}
/**
 * Start Semaphores needed for library functions
 *
 *
 * @return Negative integer if start fails.If not, returns 0 and enables semaphores
 */
int init_semaphores(){
	int rc = 0;

	rc = sem_init(&sem_valid,1,1);
	if(rc){
		printf("Error initialising semaphore valid\n");
		return rc;
	}
	rc = sem_init(&file_sync,1,1);
	if(rc){
		printf("Error initialising semaphore for GPIO files\n");
		return rc;
	}
	sem_init(&i2c_sync,1,1);
	if(rc){
		printf("Error initialising semaphore for I2C Devices\n");
		return rc;
	}
	sem_init(&alarm_sync,1,1);
	if(rc){
		printf("Error initialising semaphore for Alarm files\n");
		return rc;
	}
	rc = sem_init(&sem_hvlv,1,1);
	if(rc){
		printf("Error initialising semaphore HV LV\n");
		return rc;
	}
	rc = sem_init(&sem_dig0,1,1);
	if(rc){
		printf("Error initialising semaphore Dig0\n");
		return rc;
	}
	rc = sem_init(&sem_dig1,1,1);
	if(rc){
		printf("Error initialising semaphore Dig1\n");
		return rc;
	}
	return rc;
}

/**
 * Start Shared Memory needed for library functions
 *
 *
 * @return Negative integer if start fails.If not, returns 0 and enables shared memory
 */
int init_shared_memory() {

	int rc = 0;
	key_t sharedMemoryKey = MEMORY_KEY;

	memoryID = shmget(sharedMemoryKey, sizeof(struct wrapper), IPC_CREAT | 0600);
	if (memoryID == -1) {
		 perror("shmget(): Could not get shared memory segment");
		 return -1;
	}

	memory = static_cast<wrapper *>(shmat(memoryID, NULL, 0));
	if (memory == (void *) -1) {
		perror("shmat(): Could not map shared memory segment");
		return -1;
	}

	strcpy(memory->ev_type,"");
	strcpy(memory->ch_type,"");
	sem_init(&memory->ams_sync, 1, 0);
	sem_init(&memory->empty, 1, 1);
	sem_init(&memory->full, 1, 0);
	memory->chn = 0;
	memory->tmpstmp = 0;

	if (memoryID == -1) {
		perror("shmget(): ");
		return -1;
	 }
    return rc;
}

/**
 * Handles library closing, closing zmq context and removing sysfs GPIO folders
 *
 * @param void
 *
 * @return void
 */
void dpbsc_lib_close(struct DPB_I2cSensors *data) {
   unexport_GPIO();
   zmq_socket_destroy();
   // Release all locks
   unlink("/var/lock/LCK..ttyUL1");
   unlink("/var/lock/LCK..ttyUL2");
   unlink("/var/lock/LCK..ttyUL3");
   unlink("/var/lock/LCK..ttyUL4");
   //Destroy all semaphores
   sem_destroy(&i2c_sync);
   sem_destroy(&file_sync);
   sem_destroy(&alarm_sync);
   sem_destroy(&sem_valid);
   sem_destroy(&sem_hvlv);
   sem_destroy(&sem_dig0);
   sem_destroy(&sem_dig1);
   //Stop I2C Sensors
   stop_I2cSensors(data);
   return;
}

/** @} */

/************************** AMS Functions ******************************/
/** @defgroup ams Xilinx AMS Functions
 *  Functions to interact with Xilinx AMS SysFS to get on-chip slow control values
 *  @{
 */
/**
 * Reads temperature of n channels (channels specified in *chan) and stores the values in *res
 *
 * @param chan array which contain channels to measure
 * @param n number of channels to measure
 * @param res array where results are stored in
 *
 * @return Negative integer if reading fails.If not, returns 0 and the stored values in *res
 *
 * @note The resulting magnitude is obtained by applying the ADC conversion specified by Xilinx
 */
int xlnx_ams_read_temp(int *chan, int n, float *res){
	FILE *raw,*offset,*scale;
	for(int i=0;i<n;i++){

		char buffer [sizeof(chan[i])*8+1];
		snprintf(buffer, sizeof(buffer), "%d",chan[i]);
		char raw_str[128];
		char offset_str[128];
		char scale_str[128];

		strcpy(raw_str, "/sys/bus/iio/devices/iio:device0/in_temp");
		strcpy(offset_str, "/sys/bus/iio/devices/iio:device0/in_temp");
		strcpy(scale_str, "/sys/bus/iio/devices/iio:device0/in_temp");

		strcat(raw_str, buffer);
		strcat(offset_str, buffer);
		strcat(scale_str, buffer);

		strcat(raw_str, "_raw");
		strcat(offset_str, "_offset");
		strcat(scale_str, "_scale");

		raw = fopen(raw_str,"r");
		offset = fopen(offset_str,"r");
		scale = fopen(scale_str,"r");

		if((raw==NULL)|(offset==NULL)|(scale==NULL)){
			printf("AMS Temperature file could not be opened!!! \n");/*Any of the files could not be opened*/
			fclose(raw);
			fclose(offset);
			fclose(scale);
			return -1;
			}
		else{
			fseek(raw, 0, SEEK_END);
			long fsize = ftell(raw);
			fseek(raw, 0, SEEK_SET);  /* same as rewind(f); */

			char *raw_string = static_cast<char *>(malloc(fsize + 1));
			fread(raw_string, fsize, 1, raw);

			fseek(offset, 0, SEEK_END);
			fsize = ftell(offset);
			fseek(offset, 0, SEEK_SET);  /* same as rewind(f); */

			char *offset_string = static_cast<char *>(malloc(fsize + 1));
			fread(offset_string, fsize, 1, offset);

			fseek(scale, 0, SEEK_END);
			fsize = ftell(scale);
			fseek(scale, 0, SEEK_SET);  /* same as rewind(f); */

			char *scale_string = static_cast<char *>(malloc(fsize + 1));
			fread(scale_string, fsize, 1, scale);

			float Temperature = (atof(scale_string) * (atof(raw_string) + atof(offset_string))) / 1024; //Apply ADC conversion to Temperature, Xilinx Specs
			free(offset_string);
			free(raw_string);
			free(scale_string);
			fclose(raw);
			fclose(offset);
			fclose(scale);
			res[i] = Temperature;
			//return 0;
			}
		}
		return 0;
	}
/**
 * Reads voltage of n channels (channels specified in *chan) and stores the values in *res
 *
 * @param chan array which contain channels to measure
 * @param n number of channels to measure
 * @param res array where results are stored in
 *
 * @return Negative integer if reading fails.If not, returns 0 and the stored values in *res
 *
 * @note The resulting magnitude is obtained by applying the ADC conversion specified by Xilinx
 */
int xlnx_ams_read_volt(int *chan, int n, float *res){
	FILE *raw,*scale;
	for(int i=0;i<n;i++){

		char buffer [sizeof(chan[i])*8+1];
		snprintf(buffer, sizeof(buffer), "%d",chan[i]);
		char raw_str[128];
		char scale_str[128];

		strcpy(raw_str, "/sys/bus/iio/devices/iio:device0/in_voltage");
		strcpy(scale_str, "/sys/bus/iio/devices/iio:device0/in_voltage");

		strcat(raw_str, buffer);
		strcat(scale_str, buffer);

		strcat(raw_str, "_raw");
		strcat(scale_str, "_scale");

		raw = fopen(raw_str,"r");
		scale = fopen(scale_str,"r");

		if((raw==NULL)|(scale==NULL)){
			printf("AMS Voltage file could not be opened!!! \n");/*Any of the files could not be opened*/
			fclose(raw);
			fclose(scale);
			return -1;
			}
		else{

			fseek(raw, 0, SEEK_END);
			long fsize = ftell(raw);
			fseek(raw, 0, SEEK_SET);  /* same as rewind(f); */

			char *raw_string = static_cast<char *>(malloc(fsize + 1));
			fread(raw_string, fsize, 1, raw);

			fseek(scale, 0, SEEK_END);
			fsize = ftell(scale);
			fseek(scale, 0, SEEK_SET);  /* same as rewind(f); */

			char *scale_string = static_cast<char *>(malloc(fsize + 1));
			fread(scale_string, fsize, 1, scale);

			float Voltage = (atof(scale_string) * atof(raw_string)) / 1024; //Apply ADC conversion to Voltage, Xilinx Specs
			fclose(raw);
			fclose(scale);
			free(scale_string);
			free(raw_string);
			res[i] = Voltage;
			//return 0;
			}
		}
	return 0;
	}
/**
 * Determines the new limit of the alarm of the channel n
 *
 * @param chan channel whose alarm limit will be changed
 * @param ev_type string that determines the type of the event
 * @param ch_type string that determines the type of the channel
 * @param val value of the new limit
 *
 * @return Negative integer if setting fails, any file could not be opened or invalid argument.If not, returns 0 and the modifies the specified limit
 *
 */
int xlnx_ams_set_limits(int chan, const char *ev_type, const char *ch_type, float val){
	FILE *offset,*scale;

		char buffer [sizeof(chan)*8+1];
		char offset_str[128];
		char thres_str[128];
		char scale_str[128];
		char adc_buff [8];
		long fsize;
		int thres;
		int adc_code;
		float aux;

		if(val<0) //Cannot be negative
			return -EINVAL;

		snprintf(buffer, sizeof(buffer), "%d",chan);

		strcpy(scale_str, "/sys/bus/iio/devices/iio:device0/in_");
		strcat(scale_str, ch_type);
		strcat(scale_str, buffer);
		strcat(scale_str, "_scale");

		strcpy(thres_str, "/sys/bus/iio/devices/iio:device0/events/in_");
		strcat(thres_str, ch_type);
		strcat(thres_str, buffer);
		strcat(thres_str, "_thresh_");
		strcat(thres_str, ev_type);
		strcat(thres_str, "_value");

		scale = fopen(scale_str,"r");
		thres = open(thres_str, O_WRONLY);

		if((scale==NULL)|(thres < 0)){
			printf("AMS Voltage file could not be opened!!! \n");/*Any of the files could not be opened*/
			fclose(scale);
			return -1;
			}
		else{
			if(!strcmp("temp",ch_type)){
				strcpy(offset_str, "/sys/bus/iio/devices/iio:device0/in_");
				strcat(offset_str, ch_type);
				strcat(offset_str, buffer);
				strcat(offset_str, "_offset");
				offset = fopen(offset_str,"r");
				if(offset==NULL){
					fclose(scale);
					printf("AMS Voltage file could not be opened!!! \n");/*Any of the files could not be opened*/
					return -1;
				}
				if(strcmp("rising",ev_type)){
					fclose(scale);
					fclose(offset);
					return -EINVAL;
				}
				fseek(offset, 0, SEEK_END);
				fsize = ftell(offset);
				fseek(offset, 0, SEEK_SET);  /* same as rewind(f); */

				char *offset_string = static_cast<char *>(malloc(fsize + 1));
				fread(offset_string, fsize, 1, offset);

				fseek(scale, 0, SEEK_END);
				fsize = ftell(scale);
				fseek(scale, 0, SEEK_SET);  /* same as rewind(f); */

				char *scale_string = static_cast<char *>(malloc(fsize + 1));
				fread(scale_string, fsize, 1, scale);

				fclose(scale);
				fclose(offset);
				free(scale_string);
				free(offset_string);
				aux = (1024*val)/atof(scale_string);

			    adc_code =  (int) aux - atof(offset_string);

			}
			else if(!strcmp("voltage",ch_type)){
				if((strcmp("rising",ev_type))&(strcmp("falling",ev_type))){
					return -EINVAL;
				}
				fseek(scale, 0, SEEK_END);
				fsize = ftell(scale);
				fseek(scale, 0, SEEK_SET);  /* same as rewind(f); */

				char *scale_string = static_cast<char *>(malloc(fsize + 1));
				fread(scale_string, fsize, 1, scale);
				aux = (1024*val)/atof(scale_string);

				adc_code = (int) aux;
				free(scale_string);
				fclose(scale);

				//return 0;
			}
			else{
				close(thres);
				return -EINVAL;}

			snprintf(adc_buff, sizeof(adc_buff), "%d",adc_code);
			write (thres, &adc_buff, sizeof(adc_buff));
			close(thres);
			}
	return 0;
	}
/** @} */
/************************** I2C Devices Functions ******************************/
/** @defgroup i2c I2C Functions
 *  Functions to interact with I2C devices like sensors and other elements that can provide slow control values
 *  @{
 */
/**
 * Initialize every I2C sensor available
 *
 * @param data DPB_I2cSensors type struct which contains every I2C sensor available
 *
 * @return 0 and every I2C sensor initialized.
 */
int init_I2cSensors(struct DPB_I2cSensors *data){

	int rc;
	uint64_t timestamp;

	//Populate SFP I2C device arrays
	for (int i = 0; i < SFP_NUM; i++){

		// First page address
		strcpy(data->dev_sfp_A0[i].filename , sfp_i2c_locations[i]);
		data->dev_sfp_A0[i].addr = 0x50;

		// Second page address
		strcpy(data->dev_sfp_A2[i].filename , "/dev/i2c-6");
		data->dev_sfp_A2[i].addr = 0x51;
	}
	strcpy(data->dev_pcb_temp.filename , "/dev/i2c-2");
	data->dev_pcb_temp.addr = 0x18;

	strcpy(data->dev_som_volt.filename , "/dev/i2c-2");
	data->dev_som_volt.addr = 0x40;
	strcpy(data->dev_sfp0_2_volt.filename , "/dev/i2c-3");
	data->dev_sfp0_2_volt.addr = 0x40;
	strcpy(data->dev_sfp3_5_volt.filename , "/dev/i2c-3");
	data->dev_sfp3_5_volt.addr = 0x41;

	// PCB Temperature sensor
	sem_post(&alarm_sync);
	rc = init_tempSensor(&data->dev_pcb_temp);
	if (rc) {
		timestamp = time(NULL);
		rc = status_alarm_json("DPB","PCB Temperature Sensor I2C Bus Status",99,timestamp,"critical");
	}

	// INA 3221 voltage and current sensors
	rc = init_voltSensor(&data->dev_sfp0_2_volt);
	if (rc) {
		timestamp = time(NULL);
		rc = status_alarm_json("DPB","Voltage-Current Sensor I2C Bus Status",0,timestamp,"critical");
	}

	rc = init_voltSensor(&data->dev_sfp3_5_volt);
	if (rc) {
		timestamp = time(NULL);
		rc = status_alarm_json("DPB","Voltage-Current Sensor I2C Bus Status",1,timestamp,"critical");
	}

	rc = init_voltSensor(&data->dev_som_volt);
	if (rc) {
		timestamp = time(NULL);
		rc = status_alarm_json("DPB","Voltage-Current Sensor I2C Bus Status",2,timestamp,"critical");
	}
	// SFP I2C buses
	for (int i = 0; i < SFP_NUM; i++){
	rc = init_I2C_SFP(i,data);

	if (rc) {
			timestamp = time(NULL);
			rc = status_alarm_json("DPB","SFP I2C Bus Status",i,timestamp,"critical");
		}
	}

	// PCB Temperature set temperature limits
		rc = mcp9844_set_limits(data,0,60);
	if (rc) {
		printf("Failed to set MCP9844 Upper Limit\r\n");
	}

	rc = mcp9844_set_limits(data,2,80);
	if (rc) {
		printf("Failed to set MCP9844 Critical Limit\r\n");
	}
	return 0;
}
/**
 * Stops every I2C Sensors
 *
 * @param data DPB_I2cSensors type struct which contains every I2C sensor available
 *
 * @returns 0.
 */
int stop_I2cSensors(struct DPB_I2cSensors *data){

	i2c_stop(&data->dev_pcb_temp);

	i2c_stop(&data->dev_sfp0_2_volt);
	i2c_stop(&data->dev_sfp3_5_volt);
	i2c_stop(&data->dev_som_volt);

	for (int i = 0; i < SFP_NUM; i++){
		i2c_stop(&data->dev_sfp_A0[i]);
		i2c_stop(&data->dev_sfp_A2[i]);
	}

	return 0;
}
/** @} */
/************************** Temp.Sensor Functions ******************************/
/** @defgroup temp PCB Temperature sensor functions
 *  Functions to interact with specific configurations of the MCP9844 Temperature Sensor that measures the PCB temperature in the DPB
 *  @{
 */
/**
 * Initialize MCP9844 Temperature Sensor
 *
 * @param dev device to be initialized
 *
 * @return Negative integer if initialization fails.If not, returns 0 and the device initialized
 *
 * @note This also checks via Manufacturer and Device ID that the device is correct
 */
int init_tempSensor (struct I2cDevice *dev) {
	int rc = 0;
	uint8_t manID_buf[2] = {0,0};
	uint8_t manID_reg = MCP9844_MANUF_ID_REG;
	uint8_t devID_buf[2] = {0,0};
	uint8_t devID_reg = MCP9844_DEVICE_ID_REG;

	rc = i2c_start(dev);  //Start I2C device
		if (rc) {
			return rc;
		}
	// Write Manufacturer ID address in register pointer
	rc = i2c_write(dev,&manID_reg,1);
	if(rc < 0)
		return rc;

	// Read MSB and LSB of Manufacturer ID
	rc = i2c_read(dev,manID_buf,2);
	if(rc < 0)
			return rc;
	if(!((manID_buf[0] == 0x00) && (manID_buf[1] == 0x54))){ //Check Manufacturer ID to verify is the right component
		printf("Manufacturer ID does not match the corresponding device: Temperature Sensor\r\n");
		return -EINVAL;
	}

	// Write Device ID address in register pointer
	rc = i2c_write(dev,&devID_reg,1);
	if(rc < 0)
		return rc;

	// Read MSB and LSB of Device ID
	rc = i2c_read(dev,devID_buf,2);
	if(rc < 0)
			return rc;
	if(!((devID_buf[0] == 0x06) && (devID_buf[1] == 0x01))){//Check Device ID to verify is the right component
			printf("Device ID does not match the corresponding device: Temperature Sensor\r\n");
			return -EINVAL;
	}
	return 0;

}

/**
 * Reads ambient temperature and stores the value in *res
 *
 * @param data DPB_I2cSensors struct being the corresponding I2C device for the MCP9844 Temperature Sensor
 * @param res where the ambient temperature value is stored
 *
 * @return Negative integer if reading fails.If not, returns 0 and the stored value in *res
 *
 * @note The magnitude conversion depends if the temperature is below 0ºC or above. It also clear flag bits.
 */

int mcp9844_read_temperature(struct DPB_I2cSensors *data,float *res) {
	int rc = 0;
	struct I2cDevice dev = data->dev_pcb_temp;
	uint8_t temp_buf[2] = {0,0};
	uint8_t temp_reg = MCP9844_TEMP_REG;


	// Write temperature address in register pointer
	rc = i2c_write(&dev,&temp_reg,1);
	if(rc < 0)
		return rc;

	// Read MSB and LSB of ambient temperature
	rc = i2c_read(&dev,temp_buf,2);
	if(rc < 0)
			return rc;

	temp_buf[0] = temp_buf[0] & 0x1F;	//Clear Flag bits
	if ((temp_buf[0] & 0x10) == 0x10){//TA 0°C
		temp_buf[0] = temp_buf[0] & 0x0F; //Clear SIGN
		res[0] = (temp_buf[0] * 16 + (float)temp_buf[1] / 16) - 256; //TA 0°C
	}
	else
		res[0] = (temp_buf[0] * 16 + (float)temp_buf[1] / 16); //Temperature = Ambient Temperature (°C)
	return 0;
}
/**
 * Set alarms limits for Temperature
 *
 * @param data DPB_I2cSensors struct being the corresponding I2C device for the MCP9844 Temperature Sensor
 * @param n which limit is modified
 * @param temp_val value of the limit that is to be set
 *
 * @return Negative integer if writing fails or limit chosen is incorrect.
 * @return 0 if everything is okay and modifies the temperature alarm limit
 */
int mcp9844_set_limits(struct DPB_I2cSensors *data,int n, float temp_val) {
	int rc = 0;
	struct I2cDevice dev = data->dev_pcb_temp;
	uint8_t temp_buf[3] = {0,0,0};
	uint8_t temp_reg ;
	uint16_t temp;

	if((temp_val<-40)|(temp_val>125))
		return -EINVAL;

	switch(n){
		case MCP9844_TEMP_UPPER_LIM: //0
			temp_reg = MCP9844_TEMP_UPPER_LIM_REG;
			break;
		case MCP9844_TEMP_LOWER_LIM: //1
			temp_reg = MCP9844_TEMP_LOWER_LIM_REG;
			break;
		case MCP9844_TEMP_CRIT_LIM: //2
			temp_reg = MCP9844_TEMP_CRIT_LIM_REG;
			break;
		default:
			return -EINVAL;
	}
	if(temp_val<0){
		temp_val = -temp_val;
		temp = (short) temp_val/ 0.25;
		temp = temp << 2 ;
		temp = temp & 0x0FFF;
		temp = temp | 0x1000;
	}
	else{
		temp = (short) temp_val/ 0.25;
		temp = temp << 2 ;
		temp = temp & 0x0FFF;
	}

	temp_buf[2] = temp & 0x00FF;
	temp_buf[1] = (temp >> 8) & 0x00FF;
	temp_buf[0] = temp_reg;
	rc = i2c_write(&dev,temp_buf,3);
	if(rc < 0)
		return rc;
	return 0;
}
/**
 * Enables or disables configuration register bits of the MCP9844 Temperature Sensor
 *
 * @param data DPB_I2cSensors struct being the corresponding I2C device for the MCP9844 Temperature Sensor
 * @param bit_ena array which should contain the desired bit value (0 o 1)
 * @param bit_num array which should contain the position of the bit/s that will be modified
 *
 * @return Negative integer if writing fails,array size is mismatching or incorrect value introduced
 * @return 0 if everything is okay and modifies the configuration register
 */
int mcp9844_set_config(struct DPB_I2cSensors *data,uint8_t *bit_ena,uint8_t *bit_num) {
	int rc = 0;
	struct I2cDevice dev = data->dev_pcb_temp;
	uint8_t config_buf[2] = {0,0};
	uint8_t conf_buf[3] = {0,0,0};
	uint8_t config_reg = MCP9844_CONFIG_REG;
	uint8_t array_size = sizeof(bit_num);
	uint16_t mask;
	uint16_t config;
	if(array_size != sizeof(bit_ena))
		return -EINVAL;
	rc = i2c_write(&dev,&config_reg,1);
	if(rc < 0)
		return rc;
	// Read MSB and LSB of config reg
	rc = i2c_read(&dev,config_buf,2);
	if(rc < 0)
			return rc;
	config = (config_buf[0] << 8) + (config_buf[1]);
	for(int i = 0; i<array_size;i++){
		mask = 1;
		mask = mask << bit_num[i];
		if(bit_ena[i] == 0){
			config = config & (~mask);
		}
		else if(bit_ena[i] == 1){
			config = config | mask;
		}
		else{
			return -EINVAL;
		}
	}
	conf_buf[2] = config & 0x00FF;
	conf_buf[1] = (config >> 8) & 0x00FF;
	conf_buf[0] = config_reg;
	rc = i2c_write(&dev,conf_buf,3);
	if(rc < 0)
			return rc;
	return 0;
}

/**
 * Handles MCP9844 Temperature Sensor interruptions
 *
 * @param data DPB_I2cSensors that contains I2C Sensors
 * @param flag_buf contains alarm flags
 *
 * @return 0 and handles interruption depending on the active flags
 */
int mcp9844_interruptions(struct DPB_I2cSensors *data, uint8_t flag_buf){
	uint64_t timestamp;
	float res [1];
	int rc = 0;
	mcp9844_read_temperature(data,res);

	if((flag_buf & 0x80) == 0x80){
		timestamp = time(NULL);
		rc = alarm_json("DPB","PCB Temperature","critical", 99, res[0],timestamp,"critical");
	}
	if((flag_buf & 0x40) == 0x40){
		timestamp = time(NULL);
		rc = alarm_json("DPB","PCB Temperature","rising", 99, res[0],timestamp,"warning");
	}
	if((flag_buf & 0x20) == 0x20){
		timestamp = time(NULL);
		rc = alarm_json("DPB","PCB Temperature","falling", 99, res[0],timestamp,"warning");
	}
	return rc;
}
/**
 * Reads MCP9844 Temperature Sensor alarms flags
 *
 * @param data DPB_I2cSensors struct being the corresponding I2C device for the MCP9844 Temperature Sensor
 *
 * @return  0 and if there is any flag active calls the corresponding function to handle the interruption
 *
 * @note It also clear flag bits.
 */
int mcp9844_read_alarms(struct DPB_I2cSensors *data) {
	int rc = 0;
	struct I2cDevice dev = data->dev_pcb_temp;
	uint8_t alarm_buf[1] = {0};
	uint8_t alarm_reg = MCP9844_TEMP_REG;


	// Write temperature address in register pointer
	rc = i2c_write(&dev,&alarm_reg,1);
	if(rc < 0)
		return rc;

	// Read MSB and LSB of ambient temperature
	rc = i2c_read(&dev,alarm_buf,1);
	if(rc < 0)
			return rc;
	if((alarm_buf[0] & 0xE0) != 0){
		mcp9844_interruptions(data,alarm_buf[0]);
	}
	alarm_buf[0] = alarm_buf[0] & 0x1F;	//Clear Flag bits
	return 0;
}
/** @} */
/************************** SFP Functions ******************************/
/** @defgroup sfp SFP related functions
 *  Each SFP has an EEPROM memory that can be accessed through I2C. These functions provide several functions to read every relevant slow control variable from this EEPROM
 *  @{
 */

int init_I2C_SFP(int n, struct DPB_I2cSensors *data){
	// Check SFP powered on
	int rc = 0;
	// In case it is powered on, initialize it
		rc = init_SFP_A0(&data->dev_sfp_A0[n]);
		if (rc) {
			sfp_connected[n] = 0;
			return rc;
		}
		else{
			rc = init_SFP_A2(&data->dev_sfp_A2[n]);
			if (rc) {
				sfp_connected[n] = 0;
				return rc;
			}
			else{
				sfp_connected[n]= 1;
				return 0;
			}
		}
	return 0;
}
/**
 * Initialize SFP EEPROM page 1 as an I2C device
 *
 * @param dev I2CDevice of the SFP of which EEPROM is to be initialized
 *
 * @return Negative integer if initialization fails.If not, returns 0 and the EEPROM page initialized as I2C device
 *
 * @note This also checks via Physical device, SFP function  and the checksum registers that the device is correct and the EEPROM is working properly.
 */
int init_SFP_A0(struct I2cDevice *dev) {
	int rc = 0;
	uint8_t SFPphys_reg = SFP_PHYS_DEV;
	uint8_t SFPphys_buf[2] = {0,0};

	rc = i2c_start(dev); //Start I2C device
		if (rc) {
			return rc;
		}
	//Read SFP Physical device register
	rc = i2c_readn_reg(dev,SFPphys_reg,SFPphys_buf,1);
		if(rc < 0)
			return rc;

	//Read SFP function register
	SFPphys_reg = SFP_FUNCT;
	rc = i2c_readn_reg(dev,SFPphys_reg,&SFPphys_buf[1],1);
	if(rc < 0)
			return rc;
	if(!((SFPphys_buf[0] == 0x03) && (SFPphys_buf[1] == 0x04))){ //Check Physical device and function to verify is the right component
			printf("Device ID does not match the corresponding device: SFP-Avago\r\n");
			return -EINVAL;
	}
	rc = checksum_check(dev, SFP_PHYS_DEV,63); //Check checksum register to verify is the right component and the EEPROM is working correctly
	if(rc < 0)
				return rc;
	rc = checksum_check(dev, SFP_CHECKSUM2_A0,31);//Check checksum register to verify is the right component and the EEPROM is working correctly
	if(rc < 0)
				return rc;
	return 0;

}
/**
 * Initialize SFP EEPROM page 2 as an I2C device
 *
 * @param dev I2cDevice of the SFP of which EEPROM is to be initialized
 *
 * @return Negative integer if initialization fails.If not, returns 0 and the EEPROM page initialized as I2C device
 *
 * @note This also checks via the checksum register that the EEPROM is working properly.
 */
int init_SFP_A2(struct I2cDevice *dev) {
	int rc = 0;

	rc = i2c_start(dev);
		if (rc) {
			return rc;
		}
	rc = checksum_check(dev,SFP_MSB_HTEMP_ALARM_REG,95);//Check checksum register to verify is the right component and the EEPROM is working correctly
	if(rc < 0)
				return rc;
	return 0;

}
/**
 *Compares expected SFP checksum to its current value
 *
 * @param  dev I2cDevice of the SFP of which the checksum is to be checked
 * @param ini_reg Register where the checksum count starts
 * @param size number of registers summed for the checksum
 *
 * @return Negative integer if checksum is incorrect, and 0 if it is correct
 */
int checksum_check(struct I2cDevice *dev,uint8_t ini_reg, int size){
	int rc = 0;
	int sum = 0;
	uint8_t byte_buf[size+1] ;

	rc = i2c_readn_reg(dev,ini_reg,byte_buf,1);  //Read every register from ini_reg to ini_reg+size-1
			if(rc < 0)
				return rc;
	for(int n=1;n<(size+1);n++){
	ini_reg ++;
	rc = i2c_readn_reg(dev,ini_reg,&byte_buf[n],1);
		if(rc < 0)
			return rc;
	}

	for(int i=0;i<size;i++){
		sum += byte_buf[i];  //Sum every register read in order to obtain the checksum
	}
	uint8_t calc_checksum = (sum & 0xFF); //Only taking the 8 LSB of the checksum as the checksum register is only 8 bits
	uint8_t checksum_val = byte_buf[size];
	if (checksum_val != calc_checksum){ //Check the obtained checksum equals the device checksum register
		printf("Checksum value does not match the expected value \r\n");
		return -EHWPOISON;
	}
	return 0;
}

/**
 * Reads SFP temperature and stores the value in *res
 *
 * @param data: struct DPB_I2cSensors containing I2C devices
 * @param n indicate from which of the 6 SFP is going to be read
 * @param res where the magnitude value is stored
 *
 * @return Negative integer if reading fails.If not, returns 0 and the stored value in *res
 *
 * @note The magnitude conversion is based on the datasheet.
 */
int sfp_avago_read_temperature(struct DPB_I2cSensors *data,int n, float *res) {
	int rc = 0;
	uint8_t temp_buf[2] = {0,0};
	uint8_t temp_reg = SFP_MSB_TEMP_REG;

	struct I2cDevice dev;

	if(sfp_connected[n]){
		dev = data->dev_sfp_A2[n];
	}
	else{
		return -EINVAL;
	}

	// Read MSB of SFP temperature
	rc = i2c_readn_reg(&dev,temp_reg,temp_buf,1);
	if(rc < 0)
		return rc;

	// Read LSB of SFP temperature
	temp_reg = SFP_LSB_TEMP_REG;
	rc = i2c_readn_reg(&dev,temp_reg,&temp_buf[1],1);
	if(rc < 0)
		return rc;

	res [0] = (float) ((int) (temp_buf[0] << 8)  + temp_buf[1]) / 256;
	return 0;
}
/**
 * Reads SFP voltage supply and stores the value in *res
 *
 * @param data DPB_I2cSensors struct containing I2C devices
 * @param n indicate from which of the 6 SFP is going to be read
 * @param res where the magnitude value is stored
 *
 * @return Negative integer if reading fails.If not, returns 0 and the stored value in *res
 *
 * @note The magnitude conversion is based on the datasheet.
 */
int sfp_avago_read_voltage(struct DPB_I2cSensors *data,int n, float *res) {
	int rc = 0;
	uint8_t voltage_buf[2] = {0,0};
	uint8_t voltage_reg = SFP_MSB_VCC_REG;

	struct I2cDevice dev;

	if(sfp_connected[n]){
		dev = data->dev_sfp_A2[n];
	}
	else{
		return -EINVAL;
	}

	// Read MSB of SFP VCC
	rc = i2c_readn_reg(&dev,voltage_reg,voltage_buf,1);
	if(rc < 0)
		return rc;

	// Read LSB of SFP VCC
	voltage_reg = SFP_LSB_VCC_REG;
	rc = i2c_readn_reg(&dev,voltage_reg,&voltage_buf[1],1);
	if(rc < 0)
		return rc;

	res [0] = (float) ((uint16_t) (voltage_buf[0] << 8)  + voltage_buf[1]) * 1e-4;
	return 0;
}
/**
 * Reads SFP laser bias current and stores the value in *res
 *
 * @param data: DPB_I2cSensors struct containing I2C devices
 * @param n indicate from which of the 6 SFP is going to be read
 * @param res where the magnitude value is stored
 *
 * @return Negative integer if reading fails.If not, returns 0 and the stored value in *res
 *
 * @note The magnitude conversion is based on the datasheet.
 */
int sfp_avago_read_lbias_current(struct DPB_I2cSensors *data,int n, float *res) {
	int rc = 0;
	uint8_t current_buf[2] = {0,0};
	uint8_t current_reg = SFP_MSB_TXBIAS_REG;

	struct I2cDevice dev;

if(sfp_connected[n]){
		dev = data->dev_sfp_A2[n];
	}
	else{
		return -EINVAL;
	}
	// Read MSB of SFP Laser Bias Current
	rc = i2c_readn_reg(&dev,current_reg,current_buf,1);
	if(rc < 0)
		return rc;

	// Read LSB of SFP Laser Bias Current
	current_reg = SFP_LSB_TXBIAS_REG;
	rc = i2c_readn_reg(&dev,current_reg,&current_buf[1],1);
	if(rc < 0)
		return rc;

	res[0] = (float) ((uint16_t) (current_buf[0] << 8)  + current_buf[1]) * 2e-6;
	return 0;
}
/**
 * Reads SFP average transmitted optical power and stores the value in *res
 *
 * @param data DPB_I2cSensors containing I2C devices
 * @param n indicate from which of the 6 SFP is going to be read
 * @param res where the magnitude value is stored
 *
 * @return Negative integer if reading fails.If not, returns 0 and the stored value in *res
 *
 * @note The magnitude conversion is based on the datasheet.
 */
int sfp_avago_read_tx_av_optical_pwr(struct DPB_I2cSensors *data,int n, float *res) {
	int rc = 0;
	uint8_t power_buf[2] = {0,0};
	uint8_t power_reg = SFP_MSB_TXPWR_REG;

	struct I2cDevice dev;

if(sfp_connected[n]){
		dev = data->dev_sfp_A2[n];
	}
	else{
		return -EINVAL;
	}

	// Read MSB of SFP Laser Bias Current
	rc = i2c_readn_reg(&dev,power_reg,power_buf,1);
	if(rc < 0)
		return rc;

	// Read LSB of SFP Laser Bias Current
	power_reg = SFP_LSB_TXPWR_REG;
	rc = i2c_readn_reg(&dev,power_reg,&power_buf[1],1);
	if(rc < 0)
		return rc;

	res[0] = (float) ((uint16_t) (power_buf[0] << 8)  + power_buf[1]) * 1e-7;
	return 0;
}
/**
 * Reads SFP average received optical power and stores the value in *res
 *
 * @param data DPB_I2cSensors containing I2C devices
 * @param n indicate from which of the 6 SFP is going to be read,
 * @param res where the magnitude value is stored
 *
 * @return Negative integer if reading fails.If not, returns 0 and the stored value in *res
 *
 * @note The magnitude conversion is based on the datasheet.
 */
int sfp_avago_read_rx_av_optical_pwr(struct DPB_I2cSensors *data,int n, float *res) {
	int rc = 0;
	uint8_t power_buf[2] = {0,0};
	uint8_t power_reg = SFP_MSB_RXPWR_REG;

	struct I2cDevice dev;

if(sfp_connected[n]){
		dev = data->dev_sfp_A2[n];
	}
	else{
		return -EINVAL;
	}

	// Read MSB of SFP Laser Bias Current
	rc = i2c_readn_reg(&dev,power_reg,power_buf,1);
	if(rc < 0)
		return rc;

	// Read LSB of SFP Laser Bias Current
	power_reg = SFP_LSB_RXPWR_REG;
	rc = i2c_readn_reg(&dev,power_reg,&power_buf[1],1);
	if(rc < 0)
		return rc;

	res[0] = (float) ((uint16_t) (power_buf[0] << 8)  + power_buf[1]) * 1e-7;
	return 0;
}

/**
 * HReads SFP current RX_LOS and TX_FAULT status
 *
 * @param data DPB_I2cSensors containing I2C devices
 * @param n indicate from which of the 6 SFP is dealing with
 * @param res stores the current RX_LOS and TX_FAULT status
 *
 * @return 0 if reads properly and stores 0 or 1 depending on the current states (1 if status asserted, 0 if not)
 */
int sfp_avago_read_status(struct DPB_I2cSensors *data,int n,uint8_t *res) {
	int rc = 0;
	uint8_t status_buf[1] = {0};
	uint8_t status_reg = SFP_STAT_REG;

	struct I2cDevice dev;

if(sfp_connected[n]){
		dev = data->dev_sfp_A2[n];
	}
	else{
		return -EINVAL;
	}

	// Read status bit register
	rc = i2c_readn_reg(&dev,status_reg,status_buf,1);
	if(rc < 0)
		return rc;
	//Set RX_LOS Status Flag
	if(status_buf[0] & 0x02)
		res[0] = 1;
	else
		res[0] = 0;
	//Set TX_FAULT Status flag
	if(status_buf[0] & 0x04)
		res[1] = 1;
	else
		res[1] = 0;
	return 0;

}
/**
 * Handles SFP status interruptions
 *
 * @param status bitfields with SFP different status bits
 * @param n indicate from which of the 6 SFP is dealing with
 *
 * @return 0 and handles interruption depending on the active status flags
 */
int sfp_avago_status_interruptions(uint8_t status, int n){
	uint64_t timestamp;
	int rc = 0;

	if(((status & 0x02) != 0) & ((status_mask[n] & 0x02) == 0)){
		timestamp = time(NULL);
		rc = status_alarm_json("DPB","SFP RX_LOS Status",n,timestamp,"critical");
		status_mask[n] |= 0x02;
	}
	if(((status & 0x04) != 0) & ((status_mask[n] & 0x04) == 0)){
		timestamp = time(NULL);
		rc = status_alarm_json("DPB","SFP TX_FAULT Status", n,timestamp,"critical");
		status_mask[n] |= 0x04;
	}
	return rc;
}
/**
 * Handles SFP alarm interruptions
 *
 * @param data DPB_I2cSensors that contains I2C Sensors
 * @param flags contains alarms flags
 * @param n indicate from which of the 6 SFP is dealing with
 *
 * @return 0 and handles interruption depending on the active alarms flags
 */
int sfp_avago_alarms_interruptions(struct DPB_I2cSensors *data,uint16_t flags, int n){
	uint64_t timestamp;
	float res [1];
	int rc = 0;

	if(((flags & 0x0080) == 0x0080)&((alarms_mask[n]&0x0080)==0)){
		timestamp = time(NULL);
		sfp_avago_read_rx_av_optical_pwr(data,n,res);
		rc = alarm_json("DPB","SFP RX Power","rising", n, res[0],timestamp,"warning");
		alarms_mask[n] |= 0x0080;
	}
	if(((flags & 0x0040) == 0x0040)&((alarms_mask[n]&0x0040)==0)){
		timestamp = time(NULL);
		sfp_avago_read_rx_av_optical_pwr(data,n,res);
		rc = alarm_json("DPB","SFP RX Power","falling", n, res[0],timestamp,"warning");
		alarms_mask[n] |= 0x0040;
	}
	if(((flags & 0x0200) == 0x0200)&((alarms_mask[n]&0x0200)==0)){
		timestamp = time(NULL);
		sfp_avago_read_tx_av_optical_pwr(data,n,res);
		rc = alarm_json("DPB","SFP TX Power","rising", n, res[0],timestamp,"warning");
		alarms_mask[n] |= 0x0200;
	}
	if(((flags & 0x0100) == 0x0100)&((alarms_mask[n]&0x0100)==0)){
		timestamp = time(NULL);
		sfp_avago_read_tx_av_optical_pwr(data,n,res);
		rc = alarm_json("DPB","SFP TX Power","falling", n, res[0],timestamp,"warning");
		alarms_mask[n] |= 0x0100;
	}
	if(((flags & 0x0800) == 0x0800)&((alarms_mask[n]&0x0800)==0)){
		timestamp = time(NULL);
		sfp_avago_read_lbias_current(data,n,res);
		rc = alarm_json("DPB","SFP Laser Bias Current","rising", n, res[0],timestamp,"warning");
		alarms_mask[n] |= 0x0800;
	}
	if(((flags & 0x0400) == 0x0400)&((alarms_mask[n]&0x0400)==0)){
		timestamp = time(NULL);
		sfp_avago_read_lbias_current(data,n,res);
		rc = alarm_json("DPB","SFP Laser Bias Current","falling", n, res[0],timestamp,"warning");
		alarms_mask[n] |= 0x0400;
	}
	if(((flags & 0x2000) == 0x2000)&((alarms_mask[n]&0x2000)==0)){
		timestamp = time(NULL);
		sfp_avago_read_voltage(data,n,res);
		rc = alarm_json("DPB","SFP Voltage Monitor","rising", n, res[0],timestamp,"warning");
		alarms_mask[n] |= 0x2000;
	}
	if(((flags & 0x1000) == 0x1000)&((alarms_mask[n]&0x1000)==0)){
		timestamp = time(NULL);
		sfp_avago_read_voltage(data,n,res);
		rc = alarm_json("DPB","SFP Voltage Monitor","falling", n, res[0],timestamp,"warning");
		alarms_mask[n] |= 0x1000;
	}
	if(((flags & 0x8000) == 0x8000)&((alarms_mask[n]&0x8000)==0)){
		timestamp = time(NULL);
		sfp_avago_read_temperature(data,n,res);
		rc = alarm_json("DPB","SFP Temperature","rising", n, res[0],timestamp,"warning");
		alarms_mask[n] |= 0x8000;
	}
	if(((flags & 0x4000) == 0x4000)&((alarms_mask[n]&0x4000)==0)){
		timestamp = time(NULL);
		sfp_avago_read_temperature(data,n,res);
		// FIXME: DAQ Function Here. Replace alarm_json function by the function of DAQ library
		rc = alarm_json("DPB","SFP Temperature","falling", n, res[0],timestamp,"warning");
		alarms_mask[n] |= 0x4000;
	}

	return rc;
}
/**
 * Reads SFP status and alarms flags
 *
 * @param data DPB_I2cSensors being the corresponding I2C device for the SFP EEPROM page 2
 * @param n indicate from which of the 6 SFP is going to be read
 *
 * @return  0 and if there is any flag active calls the corresponding function to handle the interruption.
 */
int sfp_avago_read_alarms(struct DPB_I2cSensors *data,int n) {
	int rc = 0;
	uint8_t flag_buf[2] = {0,0};
	uint8_t status_buf[1] = {0};
	uint8_t status_reg = SFP_STAT_REG;
	uint8_t flag_reg = SFP_FLG1_REG;

	struct I2cDevice dev;

if(sfp_connected[n]){
		dev = data->dev_sfp_A2[n];
	}
	else{
		return -EINVAL;
	}

	// Read status bit register
	rc = i2c_readn_reg(&dev,status_reg,status_buf,1);
	if(rc < 0)
		return rc;


	// Read flag bit register 1
	rc = i2c_readn_reg(&dev,flag_reg,flag_buf,1);
	if(rc < 0)
		return rc;

	// Read flag bit register 2
	flag_reg = SFP_FLG2_REG;
	rc = i2c_readn_reg(&dev,flag_reg,&flag_buf[1],1);
	if(rc < 0)
		return rc;

	uint16_t flags =  ((uint16_t) (flag_buf[0] << 8)  + flag_buf[1]);

	if((status_buf[0] & 0x06) != 0){
		sfp_avago_status_interruptions(status_buf[0],n);
	}
	else{
		status_mask[n] = 0;
	}
	if((flags & 0xFFC0) != 0){
		sfp_avago_alarms_interruptions(data,flags,n);
	}
	else{
		alarms_mask[n] = 0;
	}
	return 0;
}
/** @} */
/************************** Volt. and Curr. Sensor Functions ******************************/
/** @defgroup inavolt INA3221 Voltage and Current Sensors
 *  DPB has voltage and current sensors that perform 4-terminal measurements in several voltage rails.
 *  The sensors employed are INA3221, a 3 channel voltage and current sensor. These functions are in charge of
 *  every operation that can be done with these sensors
 *  @{
 */

/**
 * Initialize INA3221 Voltage and Current Sensor
 *
 * @param dev I2cDevice containing device to be initialized
 *
 * @return Negative integer if initialization fails.If not, returns 0 and the device initialized
 *
 * @note This also checks via Manufacturer and Device ID that the device is correct
 */
int init_voltSensor (struct I2cDevice *dev) {
	int rc = 0;
	uint8_t manID_buf[2] = {0,0};
	uint8_t manID_reg = INA3221_MANUF_ID_REG;
	uint8_t devID_buf[2] = {0,0};
	uint8_t devID_reg = INA3221_DIE_ID_REG;

	rc = i2c_start(dev); //Start I2C device
		if (rc) {
			return rc;
		}
	// Write Manufacturer ID address in register pointer
	rc = i2c_write(dev,&manID_reg,1);
	if(rc < 0)
		return rc;

	// Read MSB and LSB of Manufacturer ID
	rc = i2c_read(dev,manID_buf,2);
	if(rc < 0)
			return rc;
	if(!((manID_buf[0] == 0x54) && (manID_buf[1] == 0x49))){ //Check Manufacturer ID to verify is the right component
		printf("Manufacturer ID does not match the corresponding device: Voltage Sensor\r\n");
		return -EINVAL;
	}

	// Write Device ID address in register pointer
	rc = i2c_write(dev,&devID_reg,1);
	if(rc < 0)
		return rc;

	// Read MSB and LSB of Device ID
	rc = i2c_read(dev,devID_buf,2);
	if(rc < 0)
			return rc;
	if(!((devID_buf[0] == 0x32) && (devID_buf[1] == 0x20))){ //Check Device ID to verify is the right component
			printf("Device ID does not match the corresponding device: Voltage Sensor\r\n");
			return -EINVAL;
	}
	return 0;

}

/**
 * Reads INA3221 Voltage and Current Sensor bus voltage from each of its 3 channels and stores the values in *res
 *
 * @param data DPB_I2cSensors struct being the corresponding I2C device INA3221 Voltage and Current Sensor
 * @param n indicate from which of the 3 INA3221 is going to be read,float *res where the voltage values are stored
 * @param res storage of collected data
 *
 * @return Negative integer if reading fails.If not, returns 0 and the stored values in *res
 *
 * @note The magnitude conversion is based on the datasheet.
 */
int ina3221_get_voltage(struct DPB_I2cSensors *data,int n, float *res){
	int rc = 0;
	uint8_t voltage_buf[2] = {0,0};
	uint8_t voltage_reg = INA3221_BUS_VOLTAGE_1_REG;
	struct I2cDevice dev;

	switch(n){
		case DEV_SFP0_2_VOLT:
			dev = data->dev_sfp0_2_volt;
		break;
		case DEV_SFP3_5_VOLT:
			dev = data->dev_sfp3_5_volt;
		break;
		case DEV_SOM_VOLT:
			dev = data->dev_som_volt;
		break;
		default:
			return -EINVAL;
		break;
	}
	for (int i=0;i<3;i++){
		if(i)
			voltage_reg = voltage_reg + 2;
		// Write bus voltage channel 1 address in register pointer
		rc = i2c_write(&dev,&voltage_reg,1);
		if(rc < 0)
			return rc;

	// Read MSB and LSB of voltage
		rc = i2c_read(&dev,voltage_buf,2);
		if(rc < 0)
			return rc;

		int voltage_int = (int)(voltage_buf[0] << 8) + (voltage_buf[1]);
		voltage_int = voltage_int / 8;
		res[i] = voltage_int * 8e-3;
	}
	return 0;
}
/**
 * Reads INA3221 Voltage and Current Sensor shunt voltage from a resistor in each of its 3 channels,
 * obtains the current dividing the voltage by the resistor value and stores the current values in *res
 *
 * @param  data DPB_I2cSensors struct being the corresponding I2C device INA3221 Voltage and Current Sensor
 * @param n indicate from which of the 3 INA3221 is going to be read,float *res where the current values are stored
 * @param res storage of collected data
 *
 * @return Negative integer if reading fails.If not, returns 0 and the stored values in *res
 *
 * @note The magnitude conversion is based on the datasheet and the resistor value is 0.05 Ohm.
 */
int ina3221_get_current(struct DPB_I2cSensors *data,int n, float *res){
	int rc = 0;
	uint8_t voltage_buf[2] = {0,0};
	uint8_t voltage_reg = INA3221_SHUNT_VOLTAGE_1_REG;
	struct I2cDevice dev;

	switch(n){
		case DEV_SFP0_2_VOLT:
				dev = data->dev_sfp0_2_volt;
		break;
		case DEV_SFP3_5_VOLT:
				dev = data->dev_sfp3_5_volt;
		break;
		case DEV_SOM_VOLT:
				dev = data->dev_som_volt;
		break;
		default:
			return -EINVAL;
		break;
		}
	for (int i=0;i<3;i++){
		if(i)
		voltage_reg = voltage_reg + 2;
		// Write shunt voltage channel 1 address in register pointer
		rc = i2c_write(&dev,&voltage_reg,1);
		if(rc < 0)
			return rc;

		// Read MSB and LSB of voltage
		rc = i2c_read(&dev,voltage_buf,2);
		if(rc < 0)
			return rc;

		int voltage_int = (int)(voltage_buf[0] << 8) + (voltage_buf[1]);
		voltage_int = voltage_int / 8;
		res[i] = (voltage_int * 40e-6) / 0.05 ;
		}
	return 0;
}
/**
 * Handles INA3221 Voltage and Current Sensor critical alarm interruptions
 *
 * @param data DPB_I2cSensors struct being the corresponding I2C device INA3221 Voltage and Current Sensor
 * @param mask contains critical alarm flags
 * @param n indicate from which of the 3 INA3221 is dealing with
 *
 * @return 0 and handles interruption depending on the active alarms flags
 */
int ina3221_critical_interruptions(struct DPB_I2cSensors *data,uint16_t mask, int n){
	uint64_t timestamp;
	float res[3];
	ina3221_get_current(data,n,res);
	int k = 0;
	int rc = 0;
	if (n == 1)
		k = 3;

	if((mask & 0x0080) == 0x0080){
		timestamp = time(NULL);
		if(n == 2)
			rc = alarm_json("DPB","Current Monitor (+12V)","rising", 99, res[2],timestamp,"critical");
		else
			rc = alarm_json("DPB","SFP Current Monitor","rising", k+2, res[2],timestamp,"critical");		}
	if((mask & 0x0100) == 0x0100){
		timestamp = time(NULL);
		if(n == 2)
			rc = alarm_json("DPB","Current Monitor (+3.3V)","rising", 99, res[2],timestamp,"critical");
		else
			rc = alarm_json("DPB","SFP Current Monitor","rising", k+2, res[2],timestamp,"critical");	}
	if((mask & 0x0200) == 0x0200){
		timestamp = time(NULL);
		if(n == 2)
			rc = alarm_json("DPB","Current Monitor (+1.8V)","rising", 99, res[2],timestamp,"critical");
		else
			rc = alarm_json("DPB","SFP Current Monitor","rising", k+2, res[2],timestamp,"critical");	}
	return rc;
}
/**
 * Handles INA3221 Voltage and Current Sensor warning alarm interruptions
 *
 * @param  data DPB_I2cSensors struct being the corresponding I2C device INA3221 Voltage and Current Sensor
 * @param  mask contains warning alarm flags
 * @param n indicate from which of the 3 INA3221 is dealing with
 *
 * @return 0 and handles interruption depending on the active alarms flags
 */
int ina3221_warning_interruptions(struct DPB_I2cSensors *data,uint16_t mask, int n){

	uint64_t timestamp;
	float res[3];
	ina3221_get_current(data,n,res);
	int k = 0;
	int rc = 0;
	if (n == 1) //Number of INA3221
		k = 3;

	if((mask & 0x0008) == 0x0008){
		timestamp = time(NULL);
		if(n == 2)
			rc = alarm_json("DPB","Current Monitor (+12V)","rising", 99, res[0],timestamp,"warning");
		else
			rc = alarm_json("DPB","SFP Current Monitor","rising", k, res[0],timestamp,"warning");	}
	if((mask & 0x0010) == 0x0010){
		timestamp = time(NULL);
		if(n == 2)
			rc = alarm_json("DPB","Current Monitor (+3.3V)","rising", 99, res[1],timestamp,"warning");
		else
			rc = alarm_json("DPB","SFP Current Monitor","rising", k+1, res[1],timestamp,"warning");	}
	if((mask & 0x0020) == 0x0020){
		timestamp = time(NULL);
		if(n == 2)
			rc = alarm_json("DPB","Current Monitor (+1.8V)","rising", 99, res[2],timestamp,"warning");
		else
			rc = alarm_json("DPB","SFP Current Monitor","rising", k+2, res[2],timestamp,"warning");	}
	return rc;
}
/**
 * Reads INA3221 Voltage and Current Sensor warning and critical alarms flags
 *
 * @param data DPB_I2cSensors struct being the corresponding I2C device for the INA3221 Voltage and Current Sensor
 * @param n indicate from which of the 3 INA3221 is going to be read
 *
 * @return  0 and if there is any flag active calls the corresponding function to handle the interruption.
 */
int ina3221_read_alarms(struct DPB_I2cSensors *data,int n){
	int rc = 0;
	uint8_t mask_buf[2] = {0,0};
	uint8_t mask_reg = INA3221_MASK_ENA_REG;
	struct I2cDevice dev;

	switch(n){
		case DEV_SFP0_2_VOLT:
			dev = data->dev_sfp0_2_volt;
		break;
		case DEV_SFP3_5_VOLT:
			dev = data->dev_sfp3_5_volt;
		break;
		case DEV_SOM_VOLT:
			dev = data->dev_som_volt;
		break;
		default:
			return -EINVAL;
		break;
		}
	// Write alarms address in register pointer
	rc = i2c_write(&dev,&mask_reg,1);
	if(rc < 0)
		return rc;

	// Read MSB and LSB of voltage
	rc = i2c_read(&dev,mask_buf,2);
	if(rc < 0)
		return rc;

	uint16_t mask_int = (uint16_t)(mask_buf[0] << 8) + (mask_buf[1]);
	if((mask_int & 0x0380)!= 0){
		ina3221_critical_interruptions(data,mask_int,n);
	}
	else if((mask_int & 0x0038)!= 0){
		ina3221_warning_interruptions(data,mask_int,n);
	}

	return 0;
}
/**
 * Set current alarms limits for INA3221 (warning or critical)
 *
 * @param data DPB_I2cSensors struct being the corresponding I2C device for the MCP9844 Temperature Sensor
 * @param n which of the 3 INA3221 is being dealt with
 * @param ch which of the 3 INA3221 channels is being dealt with
 * @param alarm_type indicates if the limit to be modifies is for a critical alarm or warning alarm
 * @param curr current value which will be the new limit
 *
 * @return Negative integer if writing fails or any parameter is incorrect.
 * @return 0 if everything is okay and modifies the current alarm limit (as shunt voltage limit)
 */
int ina3221_set_limits(struct DPB_I2cSensors *data,int n,int ch,int alarm_type ,float curr) {
	int rc = 0;
	uint8_t volt_buf[3] = {0,0,0};
	uint8_t volt_reg ;
	uint16_t volt_lim;
	struct I2cDevice dev;

	if(curr >= 1.5)
		return EINVAL;
	switch(n){
		case DEV_SFP0_2_VOLT:
			dev = data->dev_sfp0_2_volt;
		break;
		case DEV_SFP3_5_VOLT:
			dev = data->dev_sfp3_5_volt;
		break;
		case DEV_SOM_VOLT:
			dev = data->dev_som_volt;
		break;
		default:
			return -EINVAL;
		break;
		}
	switch(ch){
		case INA3221_CH1:
			volt_reg = (alarm_type)?INA3221_SHUNT_VOLTAGE_WARN1_REG:INA3221_SHUNT_VOLTAGE_CRIT1_REG;
		break;
		case INA3221_CH2:
			volt_reg = (alarm_type)?INA3221_SHUNT_VOLTAGE_WARN2_REG:INA3221_SHUNT_VOLTAGE_CRIT2_REG;
		break;
		case INA3221_CH3:
			volt_reg = (alarm_type)?INA3221_SHUNT_VOLTAGE_WARN3_REG:INA3221_SHUNT_VOLTAGE_CRIT3_REG;
		break;
		default:
			return -EINVAL;
		break;
		}
	if(curr < 0){
		curr = -curr;
		volt_lim = (curr * 0.05 * 1e6)/40; //0.05 = Resistor value
		volt_lim = volt_lim << 3 ;
		volt_lim = (~volt_lim)+1;
		volt_lim = volt_lim | 0x8000;
	}
	else{
		volt_lim = (curr * 0.05 * 1e6)/40; //0.05 = Resistor value
		volt_lim = volt_lim << 3 ;
	}
	volt_buf[2] = volt_lim & 0x00FF;
	volt_buf[1] = (volt_lim >> 8) & 0x00FF;
	volt_buf[0] = volt_reg;
	rc = i2c_write(&dev,volt_buf,3);
	if(rc < 0)
		return rc;
	return 0;
}
/**
 * Enables or disables configuration register bits of the INA3221 Voltage Sensor
 *
 * @param data DPB_I2cSensors struct being the corresponding I2C device for the INA3221 Voltage Sensor
 * @param bit_ena array which should contain the desired bit value (0 o 1)
 * @param bit_num array which should contain the position of the bit/s that will be modified
 * @param n which of the 3 INA3221 is being dealt with
 *
 * @return Negative integer if writing fails,array size is mismatching or incorrect value introduced
 * @return 0 if everything is okay and modifies the configuration register
 */
int ina3221_set_config(struct DPB_I2cSensors *data,uint8_t *bit_ena,uint8_t *bit_num, int n) {
	int rc = 0;
	uint8_t config_buf[2] = {0,0};
	uint8_t conf_buf[3] = {0,0,0};
	uint8_t config_reg = INA3221_CONFIG_REG;
	uint8_t array_size = sizeof(bit_num);
	uint16_t mask;
	uint16_t config;
	struct I2cDevice dev;

	if(array_size != sizeof(bit_ena))
		return -EINVAL;
	switch(n){
		case DEV_SFP0_2_VOLT:
			dev = data->dev_sfp0_2_volt;
		break;
		case DEV_SFP3_5_VOLT:
			dev = data->dev_sfp3_5_volt;
		break;
		case DEV_SOM_VOLT:
			dev = data->dev_som_volt;
		break;
		default:
			return -EINVAL;
		break;
		}
	rc = i2c_write(&dev,&config_reg,1);
	if(rc < 0)
		return rc;
	// Read MSB and LSB of config reg
	rc = i2c_read(&dev,config_buf,2);
	if(rc < 0)
			return rc;
	config = (config_buf[0] << 8) + (config_buf[1]);
	for(int i = 0; i<array_size;i++){
		mask = 1;
		mask = mask << bit_num[i];
		if(bit_ena[i] == 0){
			config = config & (~mask);
		}
		else if(bit_ena[i] == 1){
			config = config | mask;
		}
		else{
			return -EINVAL;
		}
	}
	conf_buf[2] = config & 0x00FF;
	conf_buf[1] = (config >> 8) & 0x00FF;
	conf_buf[0] = config_reg;
	rc = i2c_write(&dev,conf_buf,3);
	if(rc < 0)
		return rc;
	return 0;
}
/** @} */
/************************** JSON functions ******************************/
/** @defgroup json JSON related functions
 *  Slow control data is sent using JSON formatted strings. These functions are used to build
 *  JSON strings of different types (commands, monitoring, alarms...) and to interact with them to send through ZMQ to the DAQ
 *  @{
 */
/**
 * Parses monitoring float data into a JSON array so as to include it in a JSON object
 *
 * @param jsfps JSON array in which the data will be stored
 * @param sfp_num Number of measured channel (position in JSON array)
 * @param var_name Name of the measured magnitude
 * @param val Measured magnitude value in float format
 *
 * @return 0
 */
int parsing_mon_channel_data_into_object(json_object *jsfps,int sfp_num,const char *var_name, float val) {

	char buffer[512];
	struct json_object *jobj,*jdouble = NULL;
	jobj = json_object_array_get_idx(jsfps, sfp_num);
	if(jobj == NULL){
		jobj = json_object_new_object();
		sprintf(buffer, "%lf", (double) val);
		jdouble = json_object_new_double_s((double) val,buffer);
		json_object_object_add(jobj,var_name,jdouble);
		json_object_array_add(jsfps,jobj);
	}
	else{
		sprintf(buffer, "%lf", (double) val);
		jdouble = json_object_new_double_s((double) val,buffer);
		json_object_object_add(jobj,var_name,jdouble);
	}
	return 0;
}
/**
 * Parses monitoring status data into a JSON array so as to include it in a JSON object
 *
 * @param jsfps JSON array in which the data will be stored
 * @param sfp_num Number of measured channel (position in JSON array)
 * @param var_name Name of the measured magnitude
 * @param val Measured magnitude value in int format. 1 means ON, 0 means OFF
 *
 * @return 0
 */
int parsing_mon_channel_status_into_object(json_object *jsfps,int sfp_num,const char *var_name, int val) {

	char buffer[16];
	struct json_object *jobj,*jstring = NULL;
	if(val){
		strcpy(buffer,"ON");
	}
	else{
		strcpy(buffer,"OFF");
	}
	jobj = json_object_array_get_idx(jsfps, sfp_num);
	if(jobj == NULL){
		jobj = json_object_new_object();
		jstring = json_object_new_string(buffer);
		json_object_object_add(jobj,var_name,jstring);
		json_object_array_add(jsfps,jobj);
	}
	else{
		jstring = json_object_new_string(buffer);
		json_object_object_add(jobj,var_name,jstring);
	}
	return 0;
}

/**
 * Parses monitoring status data into a JSON array so as to include it in a JSON object
 *
 * @param jsfps JSON array in which the data will be stored
 * @param sfp_num Number of measured channel (position in JSON array)
 * @param var_name Name of the measured magnitude
 * @param val Measured magnitude value in string format.
 *
 * @return 0
 */
int parsing_mon_channel_string_into_object(json_object *jsfps,int sfp_num,const char *var_name, char* val) {

	struct json_object *jobj,*jstring = NULL;
	jobj = json_object_array_get_idx(jsfps, sfp_num);
	if(jobj == NULL){
		jobj = json_object_new_object();
		jstring = json_object_new_string(val);
		json_object_object_add(jobj,var_name,jstring);
		json_object_array_add(jsfps,jobj);
	}
	else{
		jstring = json_object_new_string(val);
		json_object_object_add(jobj,var_name,jstring);
	}
	return 0;
}
/**
 * Parses monitoring float data to include it directly in a JSON object
 *
 * @param jobj JSON object in which the data will be stored
 * @param var_name Name of the measured magnitude
 * @param val Measured magnitude value in float format
 *
 * @return 0
 */
int parsing_mon_environment_data_into_object(json_object *jobj,const char *var_name, float val) {

	char buffer[512];
	struct json_object *jdouble = NULL;
	sprintf(buffer, "%lf", (double) val);
	jdouble = json_object_new_double_s((double) val,buffer);
	json_object_object_add(jobj,var_name,jdouble);
	return 0;
}

/**
 * Parses monitoring status data to include it directly in a JSON object
 *
 * @param jobj JSON object in which the data will be stored
 * @param var_name Name of the measured magnitude
 * @param val Measured magnitude value in int format. 1 means ON, 0 means OFF
 *
 * @return 0
 */
int parsing_mon_environment_status_into_object(json_object *jobj,const char *var_name, int val) {

	char buffer[32];
	struct json_object *jstring = NULL;
	if(val){
		strcpy(buffer,"ON");
	}
	else{
		strcpy(buffer,"OFF");
	}
	jstring = json_object_new_string(buffer);
	json_object_object_add(jobj,var_name,jstring);
	return 0;
}

/**
 * Parses monitoring string data to include it directly in a JSON object
 *
 * @param jobj JSON object in which the data will be stored
 * @param var_name Name of the measured magnitude
 * @param val_str string to put in the value field of the JSON
 *
 * @return 0
 */
int parsing_mon_environment_string_into_object(json_object *jobj,const char *var_name, char* val_str) {

	struct json_object *jstring = NULL;
	jstring = json_object_new_string(val_str);
	json_object_object_add(jobj,var_name,jstring);
	return 0;
}

/**
 * Parses alarms data into a JSON string and send it to socket
 *

 * @param chan Number of measured channel, if chan is 99 means channel will not be parsed
 * @param val Measured magnitude value
 * @param board Board that triggered the alarm
 * @param chip Name of the chip that triggered the alarm
 * @param ev_type Type of event that has occurred
 * @param timestamp Time when the event occurred
 * @param info_type Determines the reported event type (info: warning or critical)
 *
 *
 * @return 0 or negative integer if validation fails
 */
int alarm_json (const char *board,const char *chip,const char *ev_type, int chan, float val,uint64_t timestamp,const char *info_type)
{
	sem_wait(&alarm_sync);
	struct json_object *jalarm_data,*jboard,*jchip,*jtimestamp,*jchan,*jdouble,*jev_type, *j_level = NULL;
	jalarm_data = json_object_new_object();
	char buffer[512];

	if(timestamp == 0)
		timestamp = time(NULL)*1000;

	sprintf(buffer, "%lf", (double) val);

	jboard = json_object_new_string(board);

	json_object_object_add(jalarm_data,"board", jboard);

	j_level = json_object_new_string(info_type);
	jdouble = json_object_new_double_s((double) val,buffer);
	jchip = json_object_new_string(chip);
	jev_type = json_object_new_string(ev_type);
	jtimestamp = json_object_new_int64(timestamp*1000);

	json_object_object_add(jalarm_data,"magnitudename", jchip);
	json_object_object_add(jalarm_data,"eventtype", jev_type);
	json_object_object_add(jalarm_data,"eventtimestamp", jtimestamp);
	json_object_object_add(jalarm_data,"level", j_level);

	if (chan != 99){
		jchan = json_object_new_int(chan);
		json_object_object_add(jalarm_data,"channel", jchan);
	}

	json_object_object_add(jalarm_data,"value", jdouble);


	const char *serialized_json = json_object_to_json_string(jalarm_data);
	int rc = json_schema_validate("JSONSchemaAlarms.json",serialized_json, "alarm_temp.json");
	if (rc) {
		printf("Error validating JSON Schema\r\n");
		return rc;
	}
	else{
		zmq_send(alarm_publisher, serialized_json, strlen(serialized_json), 0);
	}
	sem_post(&alarm_sync);
	json_object_put(jalarm_data);
	return 0;
}

/**
 * Parses alarms data into a JSON string and send it to socket
 *
 * @param chan Number of measured channel, if chan is 99 means channel will not be parsed (also indicates it is not SFP related)
 * @param chip Name of the chip that triggered the alarm
 * @param board Name of the board where the alarm is asserted
 * @param timestamp Time when the event occurred
 * @param info_type Determines the reported event type (info,warning or critical)
 *
 *
 * @return 0 or negative integer if validation fails
 */
int status_alarm_json (const char *board,const char *chip, int chan,uint64_t timestamp, const char *info_type)
{
	sem_wait(&alarm_sync);
	struct json_object *jalarm_data,*jboard,*jchip,*jtimestamp,*jchan,*jstatus,*j_level = NULL;
	jalarm_data = json_object_new_object();

	uint64_t timestamp_msg = (time(NULL))*1000;

	jboard = json_object_new_string(board);

	json_object_object_add(jalarm_data,"board", jboard);

	jchip = json_object_new_string(chip);
	jtimestamp = json_object_new_int64(timestamp_msg);
	j_level = json_object_new_string(info_type);

	json_object_object_add(jalarm_data,"magnitudename", jchip);
	json_object_object_add(jalarm_data,"eventtimestamp", jtimestamp);
	json_object_object_add(jalarm_data,"level", j_level);

	if (chan != 99){
		jchan = json_object_new_int(chan);
		json_object_object_add(jalarm_data,"channel", jchan);
		if((strcmp(chip,"SFP RX_LOS Status")==0)|(strcmp(chip,"SFP TX_FAULT Status")==0)){
			jstatus = json_object_new_string("ON");
		}
		else{
			jstatus = json_object_new_string("OFF");
		}
	}
	else{
		jstatus = json_object_new_string("OFF");
	}

	json_object_object_add(jalarm_data,"value", jstatus);

	const char *serialized_json = json_object_to_json_string(jalarm_data);
	int rc = json_schema_validate("JSONSchemaAlarms.json",serialized_json, "alarm_temp.json");
	if (0) {
		printf("Error validating JSON Schema\r\n");
		return -1;
	}
	else{
		zmq_send(alarm_publisher, serialized_json, strlen(serialized_json), 0);
	}
	sem_post(&alarm_sync);
	json_object_put(jalarm_data);
	return 0;
}
/**
 * Parses command response into a JSON string and send it to socket
 *
 * @param msg_id Message ID
 * @param val read value
 * @param cmd_reply Stores CMD JSON reply to send it
 *
 * @return 0 or negative integer if validation fails
 */
int command_response_json (int msg_id, float val, char* cmd_reply)
{
	json_object *jcmd_data = json_object_new_object();

	char buffer[512];
	char msg_date[64];
	char uuid[64];
	time_t t = time(NULL);
	struct tm  tms = * localtime(&t);
	struct timespec now;

	int year = tms.tm_year+1900;
	int mon  = tms.tm_mon+1;
	int day  = tms.tm_mday;
	int hour = tms.tm_hour;
	int min  = tms.tm_min;
	int sec = tms.tm_sec;

	clock_gettime( CLOCK_REALTIME, &now );
	int msec=now.tv_nsec / 1000000;

	snprintf(msg_date, sizeof(msg_date), "%d-%d-%dT%d:%d:%d.%dZ",year,mon,day,hour,min,sec,msec);

	sprintf(buffer, "%lf", (double) val);
	gen_uuid(uuid);
	json_object *jmsg_id = json_object_new_int(msg_id);
	json_object *jmsg_time = json_object_new_string(msg_date);
	json_object *jmsg_type = json_object_new_string("Command reply");
	json_object *juuid = json_object_new_string(uuid);
	json_object *jval = json_object_new_double_s((double) val,buffer);


	json_object_object_add(jcmd_data,"msg_id",jmsg_id);
	json_object_object_add(jcmd_data,"msg_time",jmsg_time);
	json_object_object_add(jcmd_data,"msg_type",jmsg_type);
	json_object_object_add(jcmd_data,"msg_value", jval);
	json_object_object_add(jcmd_data,"uuid", juuid);

	const char *serialized_json = json_object_to_json_string(jcmd_data);
	int rc = json_schema_validate("JSONSchemaSlowControl.json",serialized_json, "cmd_temp.json");
	if (rc) {
		printf("Error\r\n");
		return rc;
	}
	strcpy(cmd_reply,serialized_json);
	//zmq_send(cmd_router, serialized_json, strlen(serialized_json), 0);
	json_object_put(jcmd_data);
	return 0;
}

/**
 * Parses command response into a JSON string and send it to socket
 *
 * @param msg_id Message ID
 * @param val read value (1 is ON and 0 is OFF), if operation is set val = 99, JSON value field = OK , else is error, JSON value = ERROR
 * @param cmd_reply Stores CMD JSON reply to send it
 *
 * @return 0 or negative integer if validation fails
 */
int command_status_response_json (int msg_id,int val,char* cmd_reply)
{
	json_object *jcmd_data = json_object_new_object();
	char msg_date[64];
	char uuid[64];
	time_t t = time(NULL);
	struct tm  tms = * localtime(&t);
	struct timespec now;

	int year = tms.tm_year+1900;
	int mon  = tms.tm_mon+1;
	int day  = tms.tm_mday;
	int hour = tms.tm_hour;
	int min  = tms.tm_min;
	int sec = tms.tm_sec;

	clock_gettime( CLOCK_REALTIME, &now );
	int msec=now.tv_nsec / 1000000;

	snprintf(msg_date, sizeof(msg_date), "%d-%d-%dT%d:%d:%d.%dZ",year,mon,day,hour,min,sec,msec);
	gen_uuid(uuid);
	json_object *jval;
	json_object *jmsg_id = json_object_new_int(msg_id);
	json_object *jmsg_time = json_object_new_string(msg_date);
	json_object *jmsg_type = json_object_new_string("Command reply");
	json_object *juuid = json_object_new_string(uuid);

	if(val == 99)
		jval = json_object_new_string("OK");
	else if(val == 1)
		jval = json_object_new_string("ON");
	else if(val == 0)
		jval = json_object_new_string("OFF");
	else if (val == -2)
		jval = json_object_new_string("ERROR: SET operation not successful");
	else if (val == -3)
		jval = json_object_new_string("ERROR: READ operation not successful");
	else
		jval = json_object_new_string("ERROR: Command not valid");

	json_object_object_add(jcmd_data,"msg_id",jmsg_id);
	json_object_object_add(jcmd_data,"msg_time",jmsg_time);
	json_object_object_add(jcmd_data,"msg_type",jmsg_type);
	json_object_object_add(jcmd_data,"msg_value", jval);
	json_object_object_add(jcmd_data,"uuid", juuid);
//	const char *serialized_json1 = json_object_to_json_string(jmsg_id);
//	const char *serialized_json2 = json_object_to_json_string(jmsg_time);
//	const char *serialized_json3 = json_object_to_json_string(jmsg_type);
//	const char *serialized_json4 = json_object_to_json_string(jval);
//	const char *serialized_json5 = json_object_to_json_string(juuid);
	const char *serialized_json = json_object_to_json_string(jcmd_data);
	int rc = json_schema_validate("JSONSchemaSlowControl.json",serialized_json, "cmd_temp.json");
	if (rc) {
		printf("Error\r\n");
		return rc;
	}
	strcpy(cmd_reply,serialized_json);
	//zmq_send(cmd_router, serialized_json, strlen(serialized_json), 0);
	json_object_put(jcmd_data);
	return 0;
}

/**
 * Parses command response into a JSON string and send it to socket
 *
 * @param msg_id Message ID
 * @param val read value in string
 * @param cmd_reply Stores CMD JSON reply to send it
 *
 * @return 0 or negative integer if validation fails
 */
int command_response_string_json(int msg_id, char *val, char* cmd_reply)
{
	json_object *jcmd_data2 = json_object_new_object();
	char msg_date[64];
	char uuid[64];
	time_t t = time(NULL);
	struct tm  tms = * localtime(&t);
	struct timespec now;

	int year = tms.tm_year+1900;
	int mon  = tms.tm_mon+1;
	int day  = tms.tm_mday;
	int hour = tms.tm_hour;
	int min  = tms.tm_min;
	int sec = tms.tm_sec;

	clock_gettime( CLOCK_REALTIME, &now );
	int msec=now.tv_nsec / 1000000;

	snprintf(msg_date, sizeof(msg_date), "%d-%d-%dT%d:%d:%d.%dZ",year,mon,day,hour,min,sec,msec);

	gen_uuid(uuid);

	json_object *jmsg_id2 = json_object_new_int(msg_id);
	json_object *jmsg_time2 = json_object_new_string(msg_date);
	json_object *jmsg_type2 = json_object_new_string("Command reply");
	json_object *juuid2 = json_object_new_string(uuid);
	json_object *jval2 = json_object_new_string(val);

	json_object_object_add(jcmd_data2,"msg_id",jmsg_id2);
	json_object_object_add(jcmd_data2,"msg_time",jmsg_time2);
	json_object_object_add(jcmd_data2,"msg_type",jmsg_type2);
	json_object_object_add(jcmd_data2,"msg_value", jval2);
	json_object_object_add(jcmd_data2,"uuid", juuid2);
	const char *serialized_json = json_object_to_json_string(jcmd_data2);
	int rc = json_schema_validate("JSONSchemaSlowControl.json",serialized_json, "cmd_temp.json");
	if (rc) {
		printf("Error\r\n");
		return rc;
	}
	strcpy(cmd_reply,serialized_json);
	//zmq_send(cmd_router, serialized_json, strlen(serialized_json), 0);
	json_object_put(jcmd_data2);
	return 0;
}
/**
 * Validates generated JSON string with a validation schema
 *
 * @param schema Name of validation schema file
 * @param json_string JSON string to be validated
 * @param temp_file Name of Temporal File
 *
 * @return 0 if correct, negative integer if validation failed
 */
int json_schema_validate (const char *schema,const char *json_string, const char *temp_file)
{
	sem_wait(&sem_valid);
	FILE* fptr;
	regex_t r1;
	int data =regcomp(&r1, "document is valid.*", 0);
	char file_path[64];
	char schema_path[64];
	strcpy(file_path,"/home/petalinux/");
	strcpy(schema_path,"/home/petalinux/dpb2_json_schemas/");

	strcat(file_path,temp_file);
	strcat(schema_path,schema);
	fptr =  fopen(file_path, "a");
	if(fptr == NULL){
		regfree(&r1);
		return -EINVAL;
	}
	fwrite(json_string,sizeof(char),strlen(json_string),fptr);
	fclose(fptr);

	char command[128];
	char path[64];
	strcpy(command,"/usr/bin/json-schema-validate ");
	strcat(command,schema_path);
	strcat(command," < ");
	strcat(command,file_path);

	int  stderr_bk; //is fd for stderr backup
	stderr_bk = dup(fileno(stderr));

	int pipefd[2];
	pipe2(pipefd, 0); // (O_NONBLOCK);

	// What used to be stderr will now go to the pipe.
	dup2(pipefd[1], fileno(stderr));
	fflush(stderr);//flushall();
	/* Open the command for reading. */
	if (system(command) == -1) {
		remove(file_path);
		regfree(&r1);
		dup2(stderr_bk, fileno(stderr));//restore
		close(stderr_bk);
		close(pipefd[0]);
		close(pipefd[1]);
		sem_post(&sem_valid);
		printf("Failed to run command\n" );
		return -1;
	}
	close(pipefd[1]);
	dup2(stderr_bk, fileno(stderr));//restore
	close(stderr_bk);

	read(pipefd[0], path, 64);
	remove(file_path);
	close(pipefd[0]);

	data = regexec(&r1, path, 0, NULL, 0);
	if(data){
		printf("Error: JSON schema not valid\n" );
		regfree(&r1);
		sem_post(&sem_valid);
		return -EINVAL;
	}
	regfree(&r1);
	sem_post(&sem_valid);
	return 0;
}
/** @} */
/************************** GPIO functions ******************************/
/** @defgroup gpio GPIO related functions
 *  Some slow control data is taken by reading GPIOs through sysfs or setting vallues using other output GPIOs.
 *  These functions implement the required functionalities to use GPIO for these tasks, from creating the required sysfs directories and interacting with them.
 *  @{
 */
/**
 * Gets GPIO base address
 *
 * @param address pointer where the read GPIO base address plus corresponding offset will be stored
 *
 * @return 0
 */
int get_GPIO_base_address(int *address){

	char GPIO_dir[64] = "/sys/class/gpio/";
	regex_t r1,r2;
	DIR *dp;
	FILE *GPIO;

	int data = 0;
	int data2 = 0;
	int i = 0;
	char *arr[8];
	char label_str[64];
	struct dirent *entry;
	dp = opendir (GPIO_dir);

	data = regcomp(&r1, "gpiochip.*", 0);
	data2 = regcomp(&r2, "zynqmp_gpio.*", 0);


	while ((entry = readdir (dp)) != NULL){
		data = regexec(&r1, entry->d_name, 0, NULL, 0);
		if(data == 0){
			arr[i] = entry->d_name;
			i++;
		}
	}
	for(int j=0; j<i; j++){

		strcat(GPIO_dir,arr[j]);
		strcat(GPIO_dir,"/label");
		GPIO = fopen(GPIO_dir,"r");

		strcpy(GPIO_dir,"/sys/class/gpio/");
		fread(label_str, sizeof("zynqmp_gpio\n"), 1, GPIO);
		data2 = regexec(&r2, label_str, 0, NULL, 0);
		//fwrite(label_str, bytes, 1, stdout);
		if(data2 == 0){
		    	fclose(GPIO);
		    	strcat(GPIO_dir,arr[j]);
		    	strcat(GPIO_dir,"/base");
		    	GPIO = fopen(GPIO_dir,"r");
		    	fseek(GPIO, 0, SEEK_END);
		    	long fsize = ftell(GPIO);
		    	fseek(GPIO, 0, SEEK_SET);  /* same as rewind(f); */

		    	char *add_string = static_cast<char *>(malloc(fsize + 1));
		    	fread(add_string, fsize, 1, GPIO);
		    	address[0] = (int) atof(add_string) + 78 ;
		    	free(add_string);
				fclose(GPIO);
				break;
			}
		fclose(GPIO);
	}
	closedir(dp);
	regfree(&r1);
	regfree(&r2);
	return 0;
}

/**
 * Writes into a given GPIO address
 *
 * @param address GPIO address offset (from base address calculated from get_base_address) where the value is going to be written
 * @param value value which will be written (0 o 1)
 *
 * @return 0 if worked correctly, if not returns a negative integer.
 */
int write_GPIO(int address, int value){

	sem_wait(&file_sync);
	char cmd1[64];
	char cmd2[64];
	char dir_add[64];
	char val_add[64];
    FILE *fd1;
    FILE *fd2;
    char val[1];
    char dir[8] = "out";

    if((value != 0) && (value != 1) ){
		sem_post(&file_sync);
    	return -EINVAL;
	}

    val[0] = value + '0';
	int add = address + GPIO_BASE_ADDRESS;

    // Building first command
    snprintf(cmd1, 64, "echo %d > /sys/class/gpio/export", add);

    // Building GPIO sysfs file
    if (system(cmd1) == -1) {
		sem_post(&file_sync);
        return -EINVAL;
    }
    snprintf(dir_add, 64, "/sys/class/gpio/gpio%d/direction", add);
    snprintf(val_add, 64, "/sys/class/gpio/gpio%d/value", add);

    fd1 = fopen(dir_add,"w");
    fwrite(dir, sizeof(dir), 1,fd1);
    fclose(fd1);

    fd2 = fopen(val_add,"w");
    fwrite(val,sizeof(val), 1,fd2);
    fclose(fd2);

    // Building second command
    snprintf(cmd2, 64, "echo %d > /sys/class/gpio/unexport", add);

    //Removing GPIO sysfs file
    if (system(cmd2) == -1) {
		sem_post(&file_sync);
        return -EINVAL;
    }
	sem_post(&file_sync);
	return 0;
}

/**
 * Gets GPIO base address
 *
 * @param address GPIO address offset (from base address calculated from get_base_address) where the desired value is stored
 * @param value pointer where the read value will be stored
 *
 * @return 0 if worked correctly, if not returns a negative integer.
 */
int read_GPIO(int address,int *value){

	sem_wait(&file_sync);
	char cmd1[64];
	char cmd2[64];
	char dir_add[64];
	char val_add[64];
    FILE *fd1;
    char dir[8] = "in";
    FILE *GPIO_val;

	int add = address + GPIO_BASE_ADDRESS;
    // Building first command
    snprintf(cmd1, 64, "echo %d > /sys/class/gpio/export", add);


    // Building GPIO sysfs file
    if (system(cmd1) == -1) {
		sem_post(&file_sync);
        return -EINVAL;
    }
    snprintf(dir_add, 64, "/sys/class/gpio/gpio%d/direction", add);
    snprintf(val_add, 64, "/sys/class/gpio/gpio%d/value", add);


    fd1 = fopen(dir_add,"w");
	if(fd1 == NULL){
        sem_post(&file_sync);
        return -EINVAL;
    }
    fwrite(dir, sizeof(dir), 1,fd1);
    fclose(fd1);

    GPIO_val = fopen(val_add,"r");
	if(GPIO_val == NULL){
        sem_post(&file_sync);
        return -EINVAL;
    }
	fseek(GPIO_val, 0, SEEK_END);
	long fsize = ftell(GPIO_val);
	fseek(GPIO_val, 0, SEEK_SET);  /* same as rewind(f); */

	char *value_string = static_cast<char *>(malloc(fsize + 1));
	fread(value_string, fsize, 1, GPIO_val);
	value[0] = (int) atof(value_string);
	fclose(GPIO_val);
	free(value_string);

    // Building second command
    snprintf(cmd2, 64, "echo %d > /sys/class/gpio/unexport", add);

    //Removing GPIO sysfs file
    if (system(cmd2) == -1) {
		sem_post(&file_sync);
        return -EINVAL;
    }
	sem_post(&file_sync);
	return 0;
}

/**
 * Unexport possible remaining GPIO files when terminating app
 *
 */
void unexport_GPIO(){

	char GPIO_dir[64] = "/sys/class/gpio/";
	regex_t r1;
	DIR *dp;

	int data = 0;
	int i = 0;
	char *arr[32];
	char *num_str;
	char cmd[64];
	int GPIO_num;
	struct dirent *entry;
	dp = opendir (GPIO_dir);

	data = regcomp(&r1, "gpio4.*", 0);

	while ((entry = readdir (dp)) != NULL){
		data = regexec(&r1, entry->d_name, 0, NULL, 0);
		if(data == 0){
			arr[i] = entry->d_name;
			i++;
		}
	}
	for(int j=0; j<i; j++){
		num_str = strtok(arr[j],"gpio");
		GPIO_num = atoi(num_str);
		for(int l=0; l<GPIO_PINS_SIZE; l++){
			if(GPIO_num == (GPIO_PINS[l]+ GPIO_BASE_ADDRESS)){
				snprintf(cmd, sizeof(cmd), "echo %d > /sys/class/gpio/unexport", GPIO_num);
				system(cmd);
				break;
			}
		}
	}
	closedir(dp);
	regfree(&r1);
	return;
}


/**
 * Checks from GPIO if Ethernet Links status and reports it
 *
 * @param eth_interface Name of the Ethernet interface
 * @param status value of the Ethernet interface status
 *
 * @return  0 if parameters are OK, if not negative integer
 */
int eth_link_status (const char *eth_interface, int *status)
{
	sem_wait(&file_sync);
	char eth_link[64];
	FILE *link_file;
	char str[64];

	char cmd[64] = "ethtool ";

	strcat(cmd,eth_interface);
	strcat(cmd," | grep 'Link detected'");

	link_file = popen(cmd, "r");
	fread(eth_link, sizeof(eth_link), 1, link_file);
	pclose(link_file);

	strtok(eth_link," ");
	strtok(NULL," ");
	strcpy(str,strtok(NULL,"\n"));
	strcat(str,"");
	if((strcmp(str,"yes")) == 0)
		status[0] = 1;
	else if((strcmp(str,"no")) == 0)
		status[0] = 0;
	else{
		sem_post(&file_sync);
		return -EINVAL;
	}
	sem_post(&file_sync);
	return 0;

}

/**
 * Updates Ethernet interface status to ON/OFF
 *
 * @param eth_interface Name of the Ethernet interface
 * @param val value of the Ethernet interface status
 *
 * @return  0 if parameters are OK, if not negative integer
 */
int eth_link_status_config (char *eth_interface, int val)
{
	char eth_link[32];
	char cmd[64];
	if(strcmp(eth_interface,"ETH0") == 0){
		strcpy(eth_link,"eth0");
	}
	else{
		strcpy(eth_link,"eth1");
	}
	strcpy(cmd,"ifconfig ");
	strcat(cmd,eth_link);
	if(val == 1){
		strcat(cmd," up");
	}
	else if (val == 0){
		strcat(cmd," down");
	}
	else{
		return -EINVAL;
	}
	system(cmd);

	return 0;

}

/**
* Checks from GPIO if Ethernet Links status has changed from up to down and reports it if necessary
*
* @param str Name of the Ethernet interface
* @param flag value of the Ethernet interface flag, determines if the link was previously up
*
* @return  0 if parameters OK and reports the event, if not returns negative integer.
*/
int eth_down_alarm(const char *str,int *flag){

	int eth_status[1];
	int rc = 0;
	uint64_t timestamp ;

    if((flag[0] != 0) && (flag[0] != 1)){
    	return -EINVAL;
    }
	if((strcmp(str,"eth0")) && (strcmp(str,"eth1"))){
		return -EINVAL;}

	rc = eth_link_status(str,&eth_status[0]);
	if (rc) {
		printf("Error\r\n");
		return rc;
	}
	if((flag[0] == 0) & (eth_status[0] == 1)){
		flag[0] = eth_status[0];}
	if((flag[0] == 1) & (eth_status[0] == 0)){
		flag[0] = eth_status[0];
		if(!(strcmp(str,"eth0"))){
			timestamp = time(NULL);
			rc = status_alarm_json("DPB","Main Ethernet Link Status",99,timestamp,"critical");
			return rc;
		}
		else if(!(strcmp(str,"eth1"))){
			timestamp = time(NULL);
			rc = status_alarm_json("DPB","Backup Ethernet Link Status",99,timestamp,"critical");
			return rc;
		}
	}
	return 0;
}
/**
* Checks from GPIO if Aurora Links status has changed from up to down and reports it if necessary
 *
 * @param aurora_link Choose main or backup link of Dig0 or Dig1 (O: Dig0 Main, 1:Dig0 Backup, 2:Dig1 Main, 3:Dig1 Backup)
 * @param flag indicates current status of the link
 *
 * @return  0 if parameters are OK, if not negative integer
 */
int aurora_down_alarm(int aurora_link, int *flag){

	int aurora_status[1];
	int rc = 0;
	int address = 0;
	uint64_t timestamp ;
	char link_id[64] = "Aurora Main Link Status";

    if((flag[0] != 0) && (flag[0] != 1)){
    	return -EINVAL;
    }
	if((aurora_link>3) | (aurora_link<0)){
		return -EINVAL;}
	switch(aurora_link){
	case 0:
		address = DIG0_MAIN_AURORA_LINK;
		strcpy(link_id, "Aurora Main Link Status");
		break;
	case 1:
		address = DIG0_BACKUP_AURORA_LINK;
		strcpy(link_id, "Aurora Backup Link Status");
		break;
	case 2:
		address = DIG1_MAIN_AURORA_LINK;
		strcpy(link_id, "Aurora Main Link Status");
		break;
	case 3:
		address = DIG1_BACKUP_AURORA_LINK;
		strcpy(link_id, "Aurora Backup Link Status");
		break;
	default:
		return -EINVAL;
	}

	rc = read_GPIO(address,&aurora_status[0]);
	if (rc) {
		printf("Error\r\n");
		return rc;
	}
	if((flag[0] == 0) & (aurora_status[0] == 1)){
		flag[0] = aurora_status[0];}
	if((flag[0] == 1) & (aurora_status[0] == 0)){
		flag[0] = aurora_status[0];
		if(aurora_link<2){
			timestamp = time(NULL);
			rc = status_alarm_json("DIG0",link_id,99,timestamp,"critical");
			return rc;
		}
		else{
			timestamp = time(NULL);
			rc = status_alarm_json("DIG1",link_id,99,timestamp,"critical");
			return rc;
		}
	}
	return 0;
}
/** @} */
/************************** ZMQ Functions******************************/
/** @defgroup zmq ZMQ related functions
 *  Slow control data is sent through ZeroMQ sockets. These functions provide the ZeroMQ related functions to create and destroy the used ZMQ sockets.
 *  These functions are expected to be replaced by DAQ libraries once released
 *  @{
 */
/**
 * Initializes ZMQ monitoring, command and alarms sockets
 *
 *
 * @return 0 if parameters OK and reports the event. If not returns negative integer.
 */
int zmq_socket_init (){

	int rc = 0;
	int linger = 0;
	int sndhwm_mon_cmd = 1;
	int sndhwm_alarms = 6;
	size_t sndhwm_mon_cmd_size = sizeof(sndhwm_mon_cmd);
	size_t sndhwm_alarms_size = sizeof(sndhwm_alarms);
	size_t linger_size = sizeof(linger);

    zmq_context = zmq_ctx_new();
    mon_publisher = zmq_socket(zmq_context, ZMQ_PUB);

    zmq_setsockopt(mon_publisher, ZMQ_SNDHWM, &sndhwm_mon_cmd, sndhwm_mon_cmd_size);
    zmq_setsockopt(mon_publisher, ZMQ_RCVHWM, &sndhwm_mon_cmd, sndhwm_mon_cmd_size);
    zmq_setsockopt (mon_publisher, ZMQ_LINGER, &linger, linger_size);
    rc = zmq_bind(mon_publisher, "tcp://*:5555");
	if (rc) {
		return rc;
	}

    alarm_publisher = zmq_socket(zmq_context, ZMQ_PUB);
    zmq_setsockopt(alarm_publisher, ZMQ_SNDHWM, &sndhwm_alarms, sndhwm_alarms_size);
    zmq_setsockopt(alarm_publisher, ZMQ_RCVHWM, &sndhwm_alarms, sndhwm_alarms_size);
    zmq_setsockopt (alarm_publisher, ZMQ_LINGER, &linger, linger_size);
    rc = zmq_bind(alarm_publisher, "tcp://*:5556");
	if (rc) {
		return rc;
	}

    cmd_router = zmq_socket(zmq_context, ZMQ_REP);
    zmq_setsockopt(cmd_router, ZMQ_SNDHWM, &sndhwm_mon_cmd, sndhwm_mon_cmd_size);
    zmq_setsockopt(cmd_router, ZMQ_RCVHWM, &sndhwm_mon_cmd, sndhwm_mon_cmd_size);
    zmq_setsockopt (cmd_router, ZMQ_LINGER, &linger, linger_size);
    rc = zmq_bind(cmd_router, "tcp://*:5557");
	if (rc) {
		return rc;
	}
	return 0;
}

/**
 * Destroys ZMQ monitoring, command and alarms sockets and the context
 *
 *
 * @return 0 if succeeded to destroy ZMQ sockets. Returns errno depending on the function that failed
 */
int zmq_socket_destroy (){
	int rc = 0;
	rc = zmq_close(mon_publisher);
	if(rc){
		return errno;
	}
	rc = zmq_close(alarm_publisher);
	if(rc){
		return errno;
	}
	rc = zmq_close(cmd_router);
	if(rc){
		return errno;
	}
	rc = zmq_ctx_shutdown(zmq_context);
	if(rc){
		return errno;
	}
	rc = zmq_ctx_destroy(zmq_context);
	if(rc){
		return errno;
	}
   return rc;
}
/** @} */
/************************** Hash Tables Functions ******************************/
/** @defgroup hash Hash Table function
 *  UtHash tables, which resemble to Python Dictionaries, are used to perform the translation between the command formats of the DPB, the HV/LV and the Digitizer.
 *  The key is the DPB command and the value is the corresponding HV, LV or Digitizer command.
 *  @{
 */

/**
* Populates the HV Hash table by giving two arrays of strings corresponding to key-value pairs.
*
* @param table_size length of the table
* @param keys array of strings with the ordered keys
* @param values array of string
*
* @return 0 always
*/
int populate_hv_hash_table(int table_size, const char **keys, const char **values) {
	struct cmd_uthash *s = NULL; 
	for(int i = 0 ; i < table_size ; i++){
		s = (struct cmd_uthash *) malloc(sizeof *s);
		strcpy(s->daq_word, keys[i]);
    	strcpy(s->board_word, values[i]);
		HASH_ADD_STR(hv_cmd_table, daq_word, s);  /* id: name of key field */
	}
	return 0;
}
/**
* Populates the LV Hash table by giving two arrays of strings corresponding to key-value pairs.
*
* @param table_size length of the table
* @param keys array of strings with the ordered keys
* @param values array of string
*
* @return 0 always
*/
int populate_lv_hash_table(int table_size, const char **keys, const char **values) {
	struct cmd_uthash *s = NULL;

	for(int i = 0 ; i < table_size ; i++){
		s = (struct cmd_uthash *) malloc(sizeof *s);
		strcpy(s->daq_word, keys[i]);
    	strcpy(s->board_word, values[i]);
		HASH_ADD_STR(lv_cmd_table, daq_word, s);  /* id: name of key field */
	}
	return 0;
}

/**
* Populates the Digitizer Hash table by giving one arrays of strings corresponding to key. Values are just increasing numbers
*
* @param table_size length of the table
* @param keys array of strings with the ordered keys
*
* @return 0 always
*/
int populate_dig_hash_table(int table_size, const char **keys) {
	struct dig_uthash *s = NULL;

	for(int i = 0 ; i < table_size ; i++){
		s = (struct dig_uthash *) malloc(sizeof *s);
		strcpy(s->dpb_words, keys[i]);
    	s->dig_cmd_num = i;
		HASH_ADD_STR(dig_cmd_table, dpb_words, s);  /* id: name of key field */
	}
	return 0;
}

int get_hv_hash_table_command(char *key, char *value) {
	struct cmd_uthash *s;
	HASH_FIND_STR(hv_cmd_table,key,s);
	if(s != NULL){
		strcpy(value,s->board_word);
		return 0;
	}
	else{
		return -EINVAL;
	}
}

int get_lv_hash_table_command(char *key, char *value) {
	struct cmd_uthash *s;
	HASH_FIND_STR(lv_cmd_table,key,s);
	strcpy(value,s->board_word);
	if(s != NULL){
		strcpy(value,s->board_word);
		return 0;
	}
	else{
		return -EINVAL;
	}
}

int get_dig_hash_table_command(char **cmd, int *value) {
	struct dig_uthash *s;
	// Get Uthash table key
	char str_dpb_format[32];
	strcpy(str_dpb_format,cmd[0]);
	strcat(str_dpb_format," ");
	strcat(str_dpb_format,cmd[2]);

	if(cmd[3] != NULL && !strcmp(cmd[3],"ALL")){
		strcat(str_dpb_format," ");
		strcat(str_dpb_format,cmd[3]);
	}
	if(cmd[4] != NULL && (!strcmp(cmd[4],"ON") || !strcmp(cmd[4],"OFF"))){
		strcat(str_dpb_format," ");
		strcat(str_dpb_format,cmd[4]);
	}
	// Find the Digitizer command ID in the Uthash table
	HASH_FIND_STR(dig_cmd_table,str_dpb_format,s);
	if(s != NULL){
		*value = s->dig_cmd_num;
		return 0;
	}
	else{
		return -EINVAL;
	}
}

/**
* Helper function to detect an element inside an array
*
* @param inp element to be found in the list
* @param list list to be inspected
* @param listLen length of the list
*
* @return 1 if number found. -ENOENT if no number found
*/

int inList(int inp, int* list, int listLen) {
    int num_present = 0;
    for (int i = 0; i < listLen - 1; i++) {
        if (list[i] == inp) {
            num_present = 1;
            break;
        }
    }
    if(num_present)
    	return num_present;
    else
    	return -ENOENT;
}
/** @} */
/************************** Command handling functions ******************************/
/** @defgroup cmd Command handling Functions
 *  These functions implement the parsing and building of all the commands used in the DPB. DPB works with different sets of commands depending on the board it has to communicate with
 *  It can communicate within the DPB itself, HV, LV or Digitizers.
 *  @{
 */
/**
* Handles received DPB command
*
* @param  data: DPB_I2cSensors Struct that contains I2C devices
* @param cmd Segmented command
* @param msg_id Unique identifier of the received JSON command request message
* @param cmd_reply Stores command JSON reply to send it
*
* @return 0 if parameters OK and reports the event, if not returns negative integer.
*/
int dpb_command_handling(struct DPB_I2cSensors *data, char **cmd, int msg_id,char *cmd_reply){

	regex_t r1;
	int reg_exp;
	int rc = -1;
	int bool_set;
	int bool_read[1];
	int ams_chan[1];
	int chan ;
	float val_read[1];
	float ina3221_read[3];

	reg_exp = regcomp(&r1, "SFP.*", 0);
	reg_exp = regexec(&r1, cmd[3], 0, NULL, 0);
	if(reg_exp == 0){  //SFP
			char *sfp_num_str = strtok(cmd[3],"SFP");
			int sfp_num = atoi(sfp_num_str);

			if(strcmp(cmd[2],"STATUS") == 0){
				if(strcmp(cmd[0],"READ") == 0){
					rc = read_GPIO(SFP0_RX_LOS+(sfp_num*4),bool_read);
					if(rc){
						rc = command_status_response_json (msg_id,-ERRREAD,cmd_reply);
						goto end;
					}
					if(bool_read[0]){
						bool_read[0] = 0;
					}
					else{
						bool_read[0] = 1;
					}
					rc = command_status_response_json (msg_id,bool_read[0],cmd_reply);
					goto end;
				}
				else{
					bool_set=((strcmp(cmd[4],"ON") == 0)?(0):(1));
					rc = write_GPIO(SFP0_TX_DIS+sfp_num,bool_set);
					if(rc){
						rc = command_status_response_json (msg_id,-ERRSET,cmd_reply);
						goto end;
					}
					rc = command_status_response_json (msg_id,99,cmd_reply);
					goto end;
				}
			}
			if(strcmp(cmd[2],"VOLT") == 0){
				if(strcmp(cmd[0],"READ") == 0){
					rc = sfp_avago_read_voltage(data,sfp_num,val_read);
					if(rc){
						rc = command_status_response_json (msg_id,-ERRREAD,cmd_reply);
						goto end;
					}
					rc = command_response_json (msg_id,val_read[0],cmd_reply);
					goto end;
				}
			}
			if(strcmp(cmd[2],"CURR") == 0){
				if(strcmp(cmd[0],"READ") == 0){
					rc = ina3221_get_current(data,(sfp_num/3),ina3221_read);
					if(rc){
						rc = command_status_response_json (msg_id,-ERRREAD,cmd_reply);
						goto end;
					}
					val_read[0] = ina3221_read[sfp_num%3];
					rc = command_response_json (msg_id,val_read[0],cmd_reply);
					goto end;
				}
				else{
					rc = ina3221_set_limits(data,(sfp_num/3),sfp_num%3,1,atof(cmd[4]));
					if(rc){
						rc = command_status_response_json (msg_id,-ERRSET,cmd_reply);
						goto end;
					}
					rc = command_status_response_json (msg_id,99,cmd_reply);
					goto end;
				}
			}
			if(strcmp(cmd[2],"TEMP") == 0){
				if(strcmp(cmd[0],"READ") == 0){
					rc = sfp_avago_read_temperature(data,sfp_num,val_read);
					if(rc){
						rc = command_status_response_json (msg_id,-ERRREAD,cmd_reply);
						goto end;
					}
					rc = command_response_json (msg_id,val_read[0],cmd_reply);
					goto end;
				}
			}
			if(strcmp(cmd[2],"RXPWR") == 0){
				if(strcmp(cmd[0],"READ") == 0){
					rc = sfp_avago_read_rx_av_optical_pwr(data,sfp_num,val_read);
					if(rc){
						rc = command_status_response_json (msg_id,-ERRREAD,cmd_reply);
						goto end;
					}
					rc = command_response_json (msg_id,val_read[0],cmd_reply);
					goto end;
				}
			}
			if(strcmp(cmd[2],"TXPWR") == 0){
				if(strcmp(cmd[0],"READ") == 0){
					rc = sfp_avago_read_tx_av_optical_pwr(data,sfp_num,val_read);
					if(rc){
						rc = command_status_response_json (msg_id,-ERRREAD,cmd_reply);
						goto end;
					}
					rc = command_response_json (msg_id,val_read[0],cmd_reply);
					goto end;
				}
			}
		}
		else{
			if(strcmp(cmd[2],"STATUS") == 0){
				if(strcmp(cmd[0],"READ") == 0){
					if(strcmp(cmd[3],"ETH0") == 0){
						rc = eth_link_status("eth0",bool_read);
						if(rc){
							rc = command_status_response_json (msg_id,-ERRREAD,cmd_reply);
							goto end;
						}
						rc = command_status_response_json (msg_id,bool_read[0],cmd_reply);
						goto end;
					}
					else{
						rc = eth_link_status("eth1",bool_read);
						if(rc){
							rc = command_status_response_json (msg_id,-ERRREAD,cmd_reply);
							goto end;
						}
						rc = command_status_response_json (msg_id,bool_read[0],cmd_reply);
						goto end;
					}
				}
				else{
					bool_set=((strcmp(cmd[4],"ON") == 0)?(1):(0));
					rc = eth_link_status_config(cmd[3], bool_set);
					if(rc){
						rc = command_status_response_json (msg_id,-ERRSET,cmd_reply);
						goto end;
					}
					rc = command_status_response_json (msg_id,99,cmd_reply);
					goto end;
				}
			}
			if(strcmp(cmd[2],"VOLT") == 0){
				if(strcmp(cmd[0],"READ") == 0){
					if(strcmp(cmd[3],"FPDCPU") == 0){
						ams_chan[0] = 10;
						rc = xlnx_ams_read_volt(ams_chan,1,val_read);
						if(rc){
							rc = command_status_response_json (msg_id,-ERRREAD,cmd_reply);
							goto end;
						}
						rc = command_response_json (msg_id,val_read[0],cmd_reply);
						goto end;
					}
					else if(strcmp(cmd[3],"LPDCPU") == 0){
						ams_chan[0] = 9;
						rc = xlnx_ams_read_volt(ams_chan,1,val_read);
						if(rc){
							rc = command_status_response_json (msg_id,-ERRREAD,cmd_reply);
							goto end;
						}
						rc = command_response_json (msg_id,val_read[0],cmd_reply);
						goto end;
					}
					else{
						chan = ((strcmp(cmd[3],"12V") == 0)) ? 0 : ((strcmp(cmd[3],"3V3") == 0)) ? 1 : 2;
						rc = ina3221_get_voltage(data,2,ina3221_read);
						if(rc){
							rc = command_status_response_json (msg_id,-ERRREAD,cmd_reply);
							goto end;
						}
						val_read[0] = ((strcmp(cmd[3],"12V") == 0))? ina3221_read[0] :((strcmp(cmd[3],"3V3") == 0))? ina3221_read[1] :ina3221_read[2];
						rc = command_response_json (msg_id,val_read[0],cmd_reply);
						goto end;
					}
				}
				else{
					if(strcmp(cmd[3],"FPDCPU") == 0){
						rc = xlnx_ams_set_limits(10,"rising","voltage",atof(cmd[4]));
						if(rc){
							rc = command_status_response_json (msg_id,-ERRSET,cmd_reply);
							goto end;
						}
						rc = command_status_response_json (msg_id,99,cmd_reply);
						goto end;
					}
					else if(strcmp(cmd[3],"LPDCPU") == 0){
						rc = xlnx_ams_set_limits(9,"rising","voltage",atof(cmd[4]));
						if(rc){
							rc = command_status_response_json (msg_id,-ERRSET,cmd_reply);
							goto end;
						}
						rc = command_status_response_json (msg_id,99,cmd_reply);
						goto end;
					}
				}
			}
			if(strcmp(cmd[2],"CURR") == 0){
				if(strcmp(cmd[0],"READ") == 0){
					rc = ina3221_get_current(data,2,ina3221_read);
					if(rc){
						rc = command_status_response_json (msg_id,-ERRREAD,cmd_reply);
						goto end;
					}
					val_read[0] = ((strcmp(cmd[3],"12V") == 0))? ina3221_read[0] :((strcmp(cmd[3],"3V3") == 0))? ina3221_read[1] :ina3221_read[2];
					rc = command_response_json (msg_id,val_read[0],cmd_reply);
					goto end;
				}
				else{
					chan = ((strcmp(cmd[3],"12V") == 0)) ? 0 : ((strcmp(cmd[3],"3V3") == 0)) ? 1 : 2;
					rc = ina3221_set_limits(data,2,chan,1,atof(cmd[4]));
					if(rc){
						rc = command_status_response_json (msg_id,-ERRSET,cmd_reply);
						goto end;
					}
					rc = command_status_response_json (msg_id,99,cmd_reply);
					goto end;
				}
			}
			if(strcmp(cmd[2],"TEMP") == 0){
				if(strcmp(cmd[0],"READ") == 0){
					if(strcmp(cmd[3],"FPDCPU") == 0){
						ams_chan[0] = 8;
						rc = xlnx_ams_read_temp(ams_chan,1,val_read);
						if(rc){
							rc = command_status_response_json (msg_id,-ERRREAD,cmd_reply);
							goto end;
						}
						rc = command_response_json (msg_id,val_read[0],cmd_reply);
						goto end;
					}
					else if(strcmp(cmd[3],"LPDCPU") == 0){
						ams_chan[0] = 7;
						rc = xlnx_ams_read_temp(ams_chan,1,val_read);
						if(rc){
							rc = command_status_response_json (msg_id,-ERRREAD,cmd_reply);
							goto end;
						}
						rc = command_response_json (msg_id,val_read[0],cmd_reply);
						goto end;
					}
					else if(strcmp(cmd[3],"FPGA") == 0){
						ams_chan[0] = 20;
						rc = xlnx_ams_read_temp(ams_chan,1,val_read);
						if(rc){
							rc = command_status_response_json (msg_id,-ERRREAD,cmd_reply);
							goto end;
						}
						rc = command_response_json (msg_id,val_read[0],cmd_reply);
						goto end;
					}
					else{
						rc = mcp9844_read_temperature(data,val_read);
						if(rc){
							rc = command_status_response_json (msg_id,-ERRREAD,cmd_reply);
							goto end;
						}
						rc = command_response_json (msg_id,val_read[0],cmd_reply);
						goto end;
					}
				}
				else{
					if(strcmp(cmd[3],"FPDCPU") == 0){
						rc = xlnx_ams_set_limits(8,"rising","temp",atof(cmd[4]));
						if(rc){
							rc = command_status_response_json (msg_id,-ERRSET,cmd_reply);
							goto end;
						}
						rc = command_status_response_json (msg_id,99,cmd_reply);
						goto end;
					}
					else if(strcmp(cmd[3],"LPDCPU") == 0){
						rc = xlnx_ams_set_limits(7,"rising","temp",atof(cmd[4]));
						if(rc){
							rc = command_status_response_json (msg_id,-ERRSET,cmd_reply);
							goto end;
						}
						rc = command_status_response_json (msg_id,99,cmd_reply);
						goto end;
					}
					else if(strcmp(cmd[3],"FPGA") == 0){
						rc = xlnx_ams_set_limits(20,"rising","temp",atof(cmd[4]));
						if(rc){
							rc = command_status_response_json (msg_id,-ERRSET,cmd_reply);
							goto end;
						}
						rc = command_status_response_json (msg_id,99,cmd_reply);
						goto end;
					}
					else{
						rc = mcp9844_set_limits(data,0,atof(cmd[4]));
						if(rc){
							rc = command_status_response_json (msg_id,-ERRSET,cmd_reply);
							goto end;
						}
						rc = command_status_response_json (msg_id,99,cmd_reply);
						goto end;
					}
				}
			}
		}
	rc = command_status_response_json (msg_id,-EINCMD,cmd_reply);
end:
	regfree(&r1);
	return rc;
}

/**
 * Takes a COPacket-formatted command and sends it to the indicated digitizer in dig_num
 * through its specific serial port. This function must allocate in the future a way to use
 * the data link to receive slow control data instead of the serial port
 *
 * @param dig_num digitizer number where the command will be sent to. 0 (DIGITIZER_0) or 1 (DIGITIZER_1), corresponding to ttyUL1 and ttyUL2 respectively
 * @param cmd valid digitizer formatted command
 * @param result result of the command
 *
 * @return 0 if correct, -ETIMEDOUT if no answer is received after several retries
 */

int dig_command_handling(int dig_num, char *cmd, char *result){

	char    pkt_char = ' ';
    uint16_t index = 0;
	int serial_port_fd;
	int n;
	char read_buf[128];
	char error[128];
	char board_dev[32];
	char board_name[8];
	sem_t *sem_temp;
	strcpy(read_buf,"");
	CCOPacket pkt(COPKT_DEFAULT_START, COPKT_DEFAULT_STOP, COPKT_DEFAULT_SEP);

	switch (dig_num) {
		case DIGITIZER_0:
		strcpy(board_dev,"/dev/ttyUL1");
		strcpy(board_name,"DIG0");
		sem_temp = &sem_dig0;
		break;
		case DIGITIZER_1:
		strcpy(board_dev,"/dev/ttyUL2");
		strcpy(board_name,"DIG1");
		sem_temp = &sem_dig1;
		break;
		default:
		printf("Invalid digitizer number");
		return -EINVAL;
	}
	sem_wait(sem_temp);
	//Open one device
	serial_port_fd = open(board_dev,O_RDWR);
	if (serial_port_fd < 0) {
		//Send alarm
		printf("Error opening Dig%d UART\n",dig_num);
		sem_post(sem_temp);
		status_alarm_json("DIG0","UART Lite 3", 99,0,"warning");
		strcpy(result,"ERROR");
		return -EACCES;
	}

	// Wait until acquiring non-blocking BSD exclusive lock
	while(flock(serial_port_fd, LOCK_EX | LOCK_NB) == -1) {
		usleep(5000);
	}

	setup_serial_port(serial_port_fd);
	write(serial_port_fd, cmd, strlen(cmd));

	for(int i = 0 ; i < SERIAL_PORT_RETRIES ;){
		// Keep reading until timeout (VTIME)
		char pkt_char = ' ';
		n = read(serial_port_fd, &pkt_char, 1);
		if(n > 0){
			if(pkt_char == pkt.GetStartChar()){
				//reset
				index = 0;
			}
			read_buf[index++] = pkt_char;
		}
		else{
			//Send Warning
			printf("Warning, character not received\n");
			status_alarm_json(board_name,"Serial Port", 99,0,"warning");
			count_fails_until_success++;
			count_since_reset++;
			i++;
		}
		if(pkt_char == pkt.GetStopChar()){
			//read() doesn't add null terminated character at the end because it reads binary data
			read_buf[index] = '\0';
			count_fails_until_success = 0;
			strcpy(result,read_buf);
			goto success;
		}
	}
	//Send Critical error
	close(serial_port_fd);
	printf("Critical, character not received\n");
	status_alarm_json(board_name,"Serial Port", 99,0,"critical");
	strcpy(result,"ERROR IN Digitizer Reading");
	// Release the three locking mechanisms
	flock(serial_port_fd, LOCK_UN);
	sem_post(sem_temp);
	return -ETIMEDOUT;
success:
	close(serial_port_fd);
	// Release the three locking mechanisms
	flock(serial_port_fd, LOCK_UN);
	sem_post(sem_temp);
	return 0;
}

/**
 * Transforms DPB style command to a COPacket formatted command for digitizer.
 *
 * @param digcmd digitizer command given as a result of this function
 * @param cmd valid DPB formatted command split into words
 * @param words_n number of words of the DPB formatted command
 *
 * @return 0 if correct, -ETIMEDOUT if no answer is received after several retries
 */
int dig_command_translation(char *digcmd, char **cmd, int words_n){
	int rc = 0;
	
	int dig_cmd_id = 0;
	int value1 = 0;
	int value2 = 0;
	CCOPacket pkt(COPKT_DEFAULT_START, COPKT_DEFAULT_STOP, COPKT_DEFAULT_SEP);
	rc = get_dig_hash_table_command(cmd,&dig_cmd_id);
	if(rc){
		pkt.CreatePacket(digcmd, HkDigCmdList.CmdList[HKDIG_ERRO].CmdString);
		return -EINVAL;
	}

	//Build COPacket
	switch (dig_cmd_id){

		// Case 3 words
		// Reset the TDC counters of all channels at once
		case HKDIG_TDC_RST:
		// Reset Aurora link, both primary and secondary
		case HKDIG_RST_AURORA_LINK:
		case HKDIG_GET_PED_TYPE:
		case HKDIG_GET_GW_VER:

		// Disable all FE clearing bit in Board control register
		case HKDIG_STOP_FE_ALL:
		// Enable all FE setting bit in Board control register
		case HKDIG_START_FE_ALL:
		// Enable all FEs and all DAQ setting bits in board control register
		case HKDIG_START_DAQ_ALL:
		// Disable all FEs and all DAQ clearing bits in board control register
		case HKDIG_STOP_DAQ_ALL:

		// Run Rate monitor
		case HKDIG_RUN_RMON:
		// Get Software date and time of compilation. Use menu_str to store data
		case HKDIG_GET_SW_VER:

		case HKDIG_GET_BOARD_STATUS:

		case HKDIG_GET_BOARD_CNTRL:

		// Get TLink lock status
		case HKDIG_GET_TLNK_LOCK:

		// EEPROM commands
		case HKDIG_GET_EEPROM_OUI:			// Returns EEPROM OUI code
		case HKDIG_GET_EEPROM_EID:	

		// Get Rate monitor interval
		case HKDIG_GET_RMON_T:


		// Get uptime in seconds
		case HKDIG_GET_UPTIME:

		// Get clock selection
		case HKDIG_GET_CLOCK:

		// 3.3VA is divided by 2 (1.65V): multiply value in mV by 2
		case HKDIG_GET_BOARD_3V3A:

		// 12VA is divided by 7.8 (1.54V): multiply value in mV by 78/10
		case HKDIG_GET_BOARD_12VA:

		// I=V/2
		case HKDIG_GET_BOARD_I12V:

		// 5VA is divided by 3.85 (1.54V): multiply value in mV by 385/100
		case HKDIG_GET_BOARD_5V0A:

		// 5VF is divided by 3.85 (1.54V): multiply value in mV by 385/100
		case HKDIG_GET_BOARD_5V0F:

		// C12V is divided by 7.8 (1.54V): multiply value in mV by 78/10
		case HKDIG_GET_BOARD_C12V:

		// I of 5VF is calculated dividing read voltage by 3.6: multiply value in mV by 10/36
		case HKDIG_GET_BOARD_I5VF:

		// I of 3.3VA is calculated dividing read voltage by 3.6: multiply value in mV by 10/36
		case HKDIG_GET_BOARD_I3V3A:

		// I of 12VA is calculated dividing read voltage by 7.8: multiply value in mV by 10/78
		case HKDIG_GET_BOARD_I12VA:

		// Read Temperature from LTM84 sensors
		case HKDIG_GET_BOARD_TU40:

		case HKDIG_GET_BOARD_TU41:

		case HKDIG_GET_BOARD_TU45:

		// Get all sensors data from BME280
		case HKDIG_GET_BME_DATA:

		// Get Temperature calibration data
		case HKDIG_GET_BME_TCAL:

		// Get Pressure
		case HKDIG_GET_BME_HCAL:

		// Get pressure calibration data
		case HKDIG_GET_BME_PCAL:
			pkt.CreatePacket(digcmd, HkDigCmdList.CmdList[dig_cmd_id].CmdString);
			break;


		// Case 4 words
		case HKDIG_GET_THR_NUM:
		case HKDIG_GET_IT_NUM:
		case HKDIG_GET_DT_NUM:
		case HKDIG_EN_CAL_N:// Enable channel calibration input
		// Disable calibration input for channel n
		case HKDIG_DIS_CAL_N:
		
		// Enable FE for channel n
		case HKDIG_START_FE_N:
		// Disable FE for channel n
		case HKDIG_STOP_FE_N:
		// Start DAQ for channel n
		case HKDIG_START_DAQ_N:

		// Stop DAQ acquisition, leaving pedestal and calibration unchanged
		case HKDIG_STOP_DAQ_N:

		case HKDIG_GET_CHN_STATUS:

		case HKDIG_GET_CHN_CNTRL:

		// Set RMon interval
		case HKDIG_SET_RMON_T:

		// Return the rmon for this channel
		case HKDIG_GET_RMON_N:
		value1 = atoi(cmd[3]);
		pkt.CreatePacket(digcmd, HkDigCmdList.CmdList[dig_cmd_id].CmdString, (uint32_t)value1);
		break;


		case HKDIG_SET_THR_ALL:
		case HKDIG_SET_IT_ALL:
		case HKDIG_SET_DT_ALL:

		value1 = atoi(cmd[4]);
		pkt.CreatePacket(digcmd, HkDigCmdList.CmdList[dig_cmd_id].CmdString, (uint32_t)value1);
		break;

		//// Case 5 words
		case HKDIG_SET_THR_NUM:
		case HKDIG_SET_IT_NUM:
		case HKDIG_SET_DT_NUM:

		// Setting pedestal type per channel
		case HKDIG_SET_PED_TYPE:
		
		value1 = atoi(cmd[3]);
		value2 = atoi(cmd[4]);
		pkt.CreatePacket(digcmd, HkDigCmdList.CmdList[dig_cmd_id].CmdString, (uint32_t)value1, (uint32_t)value2);
		break;
		// Select clock
		case HKDIG_SET_CLOCK:
		if(!strcmp(cmd[3],"DPB"))
			value1 = 1;
		else {
			value1 = 0;
		}
		pkt.CreatePacket(digcmd, HkDigCmdList.CmdList[dig_cmd_id].CmdString, (uint32_t)value1);
		break;

		case HKDIG_ERRO:
			pkt.CreatePacket(digcmd, HkDigCmdList.CmdList[dig_cmd_id].CmdString);
			return -EINVAL;
			break;
		default:
			pkt.CreatePacket(digcmd, HkDigCmdList.CmdList[HKDIG_ERRO].CmdString);
			return -EINVAL;
			break;
 
	}
	// Convert back into string
	return rc;
}


/**
 * Takes a COPacket formatted response strips the answer from it and packages it
 * into a Command response type JSON
 *
 * @param board_response digitizer formatted response string
 * @param reply pointer to where the JSON will be stored
 * @param msg_id integer with a message id sent by the DAQ. Must be included in the response
 * @param cmd DPB formatted command for additional parsing
 *
 * @return always returns 0. the error is encapsulated into the JSON string to be sent to the DAQ
 */
int dig_command_response(char *board_response,char *reply,int msg_id, char **cmd){

	COPacketResponse_type	pktError=COPACKET_NOERR;
	CCOPacket pkt(COPKT_DEFAULT_START, COPKT_DEFAULT_STOP, COPKT_DEFAULT_SEP);
	
	pktError = pkt.LoadString(board_response);

	char daq_response[64];
	char digcmd[32];
	char bme_data[64];
	char *bme_value;
	char *value,*temp;
	char calT[64],calH[64],calP[64];
	float float_value;
	int  dig_num;
	int32_t tf;
	if(!strcmp(cmd[1],"DIG0")){
		dig_num = DIGITIZER_0;
		strcpy(calT,dig0_calT);
		strcpy(calH,dig0_calH);
		strcpy(calP,dig0_calP);
	}
	else{
		dig_num = DIGITIZER_1;
		strcpy(calT,dig1_calT);
		strcpy(calH,dig1_calH);
		strcpy(calP,dig1_calP);
	}
	// Get Command field of the received response
	int16_t cmdIdx = pkt.GetNextFiedlAsCOMMAND(HkDigCmdList);
	if(cmdIdx == HKDIG_ERRO){
		sprintf(daq_response,"ERROR: %s operation not successful",cmd[0]);
		command_response_string_json(msg_id,daq_response,reply);
		return 0;
	}
	if(!strcmp(cmd[0],"READ")){
		while(temp = pkt.GetNextField()) {
			value = temp;
		}
		switch (cmdIdx){
			//Float
					case HKDIG_GET_BOARD_3V3A:
					case HKDIG_GET_BOARD_12VA:
					case HKDIG_GET_BOARD_I12V:
					case HKDIG_GET_BOARD_5V0A:
					case HKDIG_GET_BOARD_5V0F:
					case HKDIG_GET_BOARD_C12V:
					case HKDIG_GET_BOARD_I5VF:
					case HKDIG_GET_BOARD_I3V3A:
					case HKDIG_GET_BOARD_I12VA:
						float_value = atof(value);
						float_value = float_value / 1000;
						command_response_json(msg_id,float_value,reply);
						break;
					// BME280 commands. Special case
					case HKDIG_GET_BME_DATA:
					case HKDIG_GET_BME_TCAL:
					case HKDIG_GET_BME_HCAL:
					case HKDIG_GET_BME_PCAL:
						pkt.CreatePacket(digcmd, HkDigCmdList.CmdList[HKDIG_GET_BME_DATA].CmdString);
						dig_command_handling(dig_num,digcmd,bme_data);
						bme280_get_temp(bme_data,calT,&tf,&float_value);
						if(!strcmp("TEMP",cmd[2])){
						command_response_json(msg_id,float_value,reply);
						}
						else{
							if(!strcmp("RELHUM",cmd[2])){
								bme280_get_relhum(bme_data,calH,&tf,&float_value);
								command_response_json(msg_id,float_value,reply);
							}
							else{
								bme280_get_press(bme_data,calP,&tf,&float_value);
								command_response_json(msg_id,float_value,reply);
							}
						}
						break;
					default:
						command_response_string_json(msg_id,value, reply);
						break;
		}
	}
	else{ // If it is SET, we just return OK in case digitizer doesnt reply with an error
		if(cmdIdx != HKDIG_ERRO){
			command_response_string_json(msg_id,"OK",reply);
		}
		else{
			command_response_string_json(msg_id,"ERROR",reply);
		}
	}

	return 0;
}

/**
 * Takes a CAEN formatted command for HV/LV and sends it through serial ports
 * Then it awaits for an answer, with a given timeout.
 *
 * @param board_dev file location of the serial port connected to HV/LV
 * @param cmd valid CAEN formatted command
 * @param result result of the command
 *
 * @return 0 if correct, -ETIMEDOUT if no answer is received after several retries
 */
int hv_lv_command_handling(char *board_dev, char *cmd, char *result){
	int serial_port_UL3;
	int n;
	char read_buf[128];
	char error[128];
	strcpy(read_buf,"");

	sem_wait(&sem_hvlv);

	//Open one device
	serial_port_UL3 = open(board_dev,O_RDWR);
	if (serial_port_UL3 < 0) {
		//Send alarm
		printf("Error opening HV/LV UART\n");
		sem_post(&sem_hvlv);
		status_alarm_json("HV/LV","UART Lite 3", 99,0,"warning");
		strcpy(result,"ERROR");
		return -EACCES;
	}
	// Wait until acquiring non-blocking BSD exclusive lock
	while(flock(serial_port_UL3, LOCK_EX | LOCK_NB) == -1) {
		usleep(5000);
	}

	setup_serial_port(serial_port_UL3);
	write(serial_port_UL3, cmd, strlen(cmd));
	// Try with UL3
	for(int i = 0 ; i < SERIAL_PORT_RETRIES ;){
		// Keep reading until timeout (VTIME)
		char temp_buf[8];
		n = read(serial_port_UL3, temp_buf, sizeof(temp_buf));
		if(n > 0){
			//read() doesn't add null terminated character at the end because it reads binary data
			temp_buf[n] = '\0';
			strcat(read_buf,temp_buf);
		}
		else{
			//Send Warning
			printf("Warning, character not received\n");
			status_alarm_json("HV/LV","UART Lite 3", 99,0,"warning");
			count_fails_until_success++;
			count_since_reset++;
			i++;
		}
		if(temp_buf[n-1] == '\n'){
			count_fails_until_success = 0;
			strcpy(result,read_buf);
			goto success;
		}
	}
	//Send Critical error
	close(serial_port_UL3);
	printf("Critical, character not received\n");
	status_alarm_json("HV/LV","UART Lite 3", 99,0,"critical");
	strcpy(result,"ERROR IN HV/LV Reading");
	// Release the two locking mechanisms
	flock(serial_port_UL3, LOCK_UN);
	sem_post(&sem_hvlv);
	return -ETIMEDOUT;
success:
	close(serial_port_UL3);
	// Release the two locking mechanisms
	flock(serial_port_UL3, LOCK_UN);
	sem_post(&sem_hvlv);
	return 0;
}

/**
 * Transforms DPB style command to a CAEN formatted command for HV/LV.
 *
 * @param hvlvcmd Beginning of the command string for CAEN command, to distinguish between HV/LV
 * Should be "$BD:0/1,$CMD:"
 * @param cmd valid DPB formatted command split into words
 * @param words_n number of words of the DPB formatted command
 *
 * @return 0 if correct, -ETIMEDOUT if no answer is received after several retries
 */
int hv_lv_command_translation(char *hvlvcmd, char **cmd, int words_n){
	int rc = 0;
	char chancode[8] = "CH:";
	if(!strcmp(cmd[0],"READ")){
		strcat(hvlvcmd,"MON,");
	}
	else{
		strcat(hvlvcmd,"SET,");
	}
	if(words_n >=4){
		strcat(chancode,cmd[3]);
		strcat(hvlvcmd,chancode);
		strcat(hvlvcmd,",");
	}
	strcat(hvlvcmd,"PAR:");
	char opcode[8];
	if(!strcmp(cmd[1],"LV")){
		if(!strcmp(cmd[2],"STATUS") && (!strcmp(cmd[3],"0") || !strcmp(cmd[3],"1"))){
			strcpy(opcode, "BCEN");
		}
		else{
			rc = get_lv_hash_table_command(cmd[2],opcode);
			if(rc){
				strcpy(hvlvcmd,"ERROR");
				return -EINVAL;
			}
		}
	}
	else{
		if(!strcmp(cmd[0],"SET") && !strcmp(cmd[2],"VOLT") ){
			strcpy(opcode, "VSET");
		}
		else if(!strcmp(cmd[0],"SET") && !strcmp(cmd[2],"CURR") ){
			strcpy(opcode, "ISET");
		}
		else if(!strcmp(cmd[0],"SET") && !strcmp(cmd[2],"STATUS") ){
			strcpy(opcode, "PW");
		}
		else {
			rc = get_hv_hash_table_command(cmd[2],opcode);
			if(rc){
				strcpy(hvlvcmd,"ERROR");
				return -EINVAL;
			}

		}
	}
	strcat(hvlvcmd,opcode);
	if(words_n==5){
		strcat(hvlvcmd,",VAL:");
		strcat(hvlvcmd,cmd[4]);
	}
	strcat(hvlvcmd,"\r\n");
	return 0;

}

/**
 * Takes a CAEN formatted response for HV/LV, strips the answer from it and packages it
 * into a Command response type JSON
 *
 * @param board_response CAEN formatted response string
 * @param reply pointer to where the JSON will be stored
 * @param msg_id integer with a message id sent by the DAQ. Must be included in the response
 * @param cmd DPB formatted command for additional parsing
 *
 * @return always returns 0. the error is encapsulated into the JSON string to be sent to the DAQ
 */
int hv_lv_command_response(char *board_response,char *reply,int msg_id, char **cmd){
	// Strip the returned value from response string
	char mag_str[64];
	char *target = NULL;
	char *start, *end;
	char start_string[32];
	if((!strcmp(cmd[1],"LV") && !lv_connected) || (!strcmp(cmd[1],"HV") && !hv_connected )){
		command_response_string_json(msg_id,board_response,reply);
		return 0;
	}
	if(!strcmp(cmd[0],"READ")){
		strcpy(start_string,"#CMD:OK,VAL:");
	}
	else{
		strcpy(start_string,"#CMD:OK");
	}

	if ( (start = strstr( board_response, start_string )) ){
		start += strlen( start_string );
		if ( (end = strstr( start, "\r\n" )) )
		{
			target = ( char * )malloc( end - start + 1 );
			if(target){
				memcpy( target, start, end - start );
				target[end - start] = '\0';
				strcpy(mag_str,target);
				free(target);
			}
			else {
				if(!strcmp(cmd[0],"READ")){
					strcpy(mag_str,"ERROR: READ operation not successful");
				}
				else {
					strcpy(mag_str,"ERROR: SET operation not successful");
				}
				goto end;
			}

		}
		else {
			if(!strcmp(cmd[0],"READ")){
				strcpy(mag_str,"ERROR: READ operation not successful");
			}
			else {
				strcpy(mag_str,"ERROR: SET operation not successful");
			}
			goto end;
		}
	}
	else {
		if(!strcmp(cmd[0],"READ")){
			strcpy(mag_str,"ERROR: READ operation not successful");
		}
		else {
			strcpy(mag_str,"ERROR: SET operation not successful");
		}
		goto end;
	}

	if(!strcmp(cmd[0],"SET"))
		strcpy(mag_str,"OK");
	else if(!strcmp(cmd[1],"HV") && !strcmp(cmd[2],"STATUS")){
		int mag_status = atoi(mag_str) & 0x1;
		if(mag_status){
			strcpy(mag_str,"ON");
		}
		else{
			strcpy(mag_str,"OFF");
		}
	}
	else if(!strcmp(cmd[1],"HV") && !strcmp(cmd[2],"CHANERR")){
		int mag_status = atoi(mag_str) & (0x1 << 13);
		if(mag_status){
			strcpy(mag_str,"ON");
		}
		else{
			strcpy(mag_str,"OFF");
		}
	}
end:	command_response_string_json(msg_id,mag_str,reply);
		return 0;
}

/**
 * Setups a given serial port with the standard 115200, 8 bits 1 stop bit no parity and flow control disabled
 *
 * @param serial_port file descriptor of the already opened serial port
 *
 * @return 0 if correct, -1 if failed to set the attributes
 */
int setup_serial_port(int serial_port){

	struct termios tty;
	if(tcgetattr(serial_port, &tty) != 0) {
    	printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
		return -1;
	}
	tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
	tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
	tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
	tty.c_cflag |= CS8; // 8 bits per byte (most common)
	tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
	tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

	tty.c_lflag &= ~ICANON;
	tty.c_lflag &= ~ECHO; // Disable echo
	tty.c_lflag &= ~ECHOE; // Disable erasure
	tty.c_lflag &= ~ECHONL; // Disable new-line echo
	tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

	tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

	tty.c_cc[VTIME] = SERIAL_PORT_TIMEOUT; // Set VTIME to the read timeout specified in the macro
	tty.c_cc[VMIN] = 0;
	cfsetispeed(&tty, B115200);
	cfsetospeed(&tty, B115200);

	if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    	printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
		return -1;
	}

	//Set the serial port in low latency mode

	struct serial_struct serial_settings;
	ioctl(serial_port, TIOCGSERIAL, &serial_settings);
	serial_settings.flags |= ASYNC_LOW_LATENCY;
	ioctl(serial_port, TIOCSSERIAL, &serial_settings);

	return 0;
}

/**
 * Reads HV related alarms and sends the corresponding alert message if any of the alarms is detected.
 * Currently, the alarms from the HV being read are overvoltage, overcurrent, undervoltage and TRIP
 *
 * @return 0 if correct, negative number if failed to send JSON alarm
 */
int hv_read_alarms(){
	// // We just read the Status register from the HV
	char board_dev[16];
	char hvlvcmd[40];
	char buffer[8];
	char response[40];
	char mag_str[32];
	int rc = 0;
	int OVC_flag, OVV_flag, UNV_flag, TRIP_flag;
	strcpy(board_dev,"/dev/ttyUL3");
	//Get Timestamp
	uint64_t timestamp;
	timestamp = time(NULL);

	//Parse all channels
	for(int i = 0 ; i < 24; i++){
		strcpy(hvlvcmd,"$BD:1,$CMD:MON,CH:");
		sprintf(buffer, "%d",i);
		strcat(hvlvcmd,buffer);
		strcat(hvlvcmd,",PAR:STATUS\r\n");
		hv_lv_command_handling(board_dev,hvlvcmd,response);
		char *target = NULL;
		char *start, *end;
		if ( (start = strstr( response, "#CMD:OK,VAL:" ) )){
			start += strlen( "#CMD:OK,VAL:" );
			if ( (end = strstr( start, "\r\n" )) )
			{
				target = ( char * )malloc( end - start + 1 );
				if(target){
					memcpy( target, start, end - start );
					target[end - start] = '\0';
					strcpy(mag_str,target);
				}
				else{
					strcpy(mag_str,"ERROR");
				}
				free(target);
			}
			else {
				rc = -EINVAL;
				strcpy(mag_str,"ERROR");
				return rc;
			}
		}
		else {
				rc = -EINVAL;
				strcpy(mag_str,"ERROR");
				return rc;
		}
		//Get overcurrent, overvoltage, undervoltage and trip bit flags
		OVC_flag = atoi(mag_str) & BIT(3);
		if(OVC_flag)
			rc = status_alarm_json("HV","Overcurrent",i,timestamp,"critical");
		OVV_flag = atoi(mag_str) & BIT(4);
		if(OVV_flag)
			rc = status_alarm_json("HV","Overvoltage",i,timestamp,"critical");
		UNV_flag = atoi(mag_str) & BIT(5);
		if(UNV_flag)
			rc = status_alarm_json("HV","Undervoltage",i,timestamp,"critical");
		TRIP_flag = atoi(mag_str) & BIT(6);
		if(TRIP_flag)
			rc = status_alarm_json("HV","TRIP",i,timestamp,"critical");
	}
	return rc;

}
/** @} */


/************************** Other functions ******************************/
/** @defgroup other Other Functions
 *  Miscellaneous functions
 *  @{
 */
/**
 * Generates UUID
 *
 * @param uuid String where UUID is stored
 *
 * @return 0 and stores the UUID generated
 */
int gen_uuid(char *uuid) {
    char v[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};
    //8 dash 4 dash 4 dash 12
    static char buf[33] = {0};

    //gen random for all spaces
    for(int i = 0; i < 32; ++i) {
        buf[i] = v[rand()%16];
    }

    //put dashes in place
    buf[8] = '-';
    buf[13] = '-';
    buf[18] = '-';

    //needs end byte
    buf[32] = '\0';
    strcpy(uuid,buf);

    return 0;
}

/**
 * Read from Shared memory to get IIO_EVENT_MONITOR Events
 *
 * @param channel Channel that triggered the alarm
 * @param ev_type Direction of event triggered
 * @param ch_type Type of channel that triggered the event
 *
 * @return 0
 */
int read_shm(int *channel, char *ev_type, char *ch_type){
	memoryID = shmget(MEMORY_KEY, sizeof(struct wrapper), 0);
	if (memoryID == -1) {
	      perror("Shared memory");
	      return 1;
	   }

	memory = static_cast<wrapper *>(shmat(memoryID, NULL, 0));
	channel[0] = memory->chn;
	strcpy(ev_type,memory->ev_type);
	strcpy(ch_type,memory->ch_type);
	 if (shmdt(memory) == -1) {
	      perror("shmdt");
	      return 1;
	   }
	return 0;
}
/**
 * Get Temperature measurement from BME280 (in the digitizer)
 *
 * @param data Data string from BME280
 * @param cal Calibration temperature string from BME280
 *
 * @return 0
 */
int bme280_get_temp(char *data,char *cal,int32_t *tf, float *temp){
	
	char temp_data[16];
	char substr[5];
	// Get temperature raw data and calibration from BME sampled data
	strncpy(temp_data,data+6,6);
	temp_data[6] = '\0';

	// Convert data and calibration
	int32_t data_int;
	unsigned short cal_T1;
	signed short cal_T2;
	signed short cal_T3;
	sscanf(temp_data,"%x",&data_int);
	for(int i = 0; i < 3; i++){
		// Switch endianness
		substr[2] = cal[4*i];
		substr[3] = cal[4*i + 1];
		substr[0] = cal[4*i + 2];
		substr[1] = cal[4*i + 3];
		substr[4] = '\0';
		switch(i){
			case 0:
			sscanf(substr,"%hx",&cal_T1);
			break;
			case 1:
			sscanf(substr,"%hx",&cal_T2);
			break;
			case 2:
			sscanf(substr,"%hx",&cal_T3);
			break;
		}
	}
	data_int = data_int>>4;

	// Perform the compensation calculations
	int32_t v1,v2;
	v1 = ((((data_int>>3) - ((int32_t)cal_T1<<1))) * ((int32_t)cal_T2)) >> 11;
  	v2 = (((((data_int>>4)-((int32_t)cal_T1)) * ((data_int>>4)-((int32_t)cal_T1))) >> 12) * ((int32_t)cal_T3)) >> 14;
  	tf[0] = v1+v2;
	int32_t temp_int = (tf[0]*5+128) >> 8;
  	temp[0] = (float) (temp_int*0.01);

	return 0;
}

/**
 * Get Pressure measurement from BME280 (in the digitizer)
 *
 * @param data Data string from BME280
 * @param cal Calibration pressure string from BME280
 * @param tf Temperature parameter needed for pressure calculation
 * @param press Pressure computed
 *
 * @return 0
 */
int bme280_get_press(char *data,char *cal,int32_t *tf,float *press){
	
	char press_data[16];
	char substr[5];
	// Get temperature raw data and calibration from BME sampled data
	strncpy(press_data,data,6);
	press_data[6] = '\0';

	// Convert data and calibration
	int32_t data_int;
	unsigned short cal_P1;
	signed short cal_P2,cal_P3,cal_P4,cal_P5,cal_P6,cal_P7,cal_P8,cal_P9;

	sscanf(press_data,"%x",&data_int);
	for(int i = 0; i < 9; i++){
		// Switch endianness
		substr[2] = cal[4*i];
		substr[3] = cal[4*i + 1];
		substr[0] = cal[4*i + 2];
		substr[1] = cal[4*i + 3];
		substr[4] = '\0';
		switch(i){
			case 0:
			sscanf(substr,"%hx",&cal_P1);
			break;
			case 1:
			sscanf(substr,"%hx",&cal_P2);
			break;
			case 2:
			sscanf(substr,"%hx",&cal_P3);
			break;
			case 3:
			sscanf(substr,"%hx",&cal_P4);
			break;
			case 4:
			sscanf(substr,"%hx",&cal_P5);
			break;
			case 5:
			sscanf(substr,"%hx",&cal_P6);
			break;
			case 6:
			sscanf(substr,"%hx",&cal_P7);

			break;
			case 7:
			sscanf(substr,"%hx",&cal_P8);
			break;
			case 8:
			sscanf(substr,"%hx",&cal_P9);
			break;
		}
	}
	data_int = data_int>>4;

	// Perform the compensation calculations
	int64_t v1_1,v2_1,v2_2,v2_3,v1_2,v1_3,p1,p2,v1_4,v2_4,p3;
	v1_1 = ((int64_t)tf[0])-128000;
	v2_1 = v1_1 * v1_1 * (int64_t)cal_P6;
	v2_2 = v2_1 + ((v1_1*(int64_t)cal_P5)<<17);
	v2_3 = v2_2 + (((int64_t)cal_P4)<<35);
	v1_2 = ((v1_1 * v1_1 * (int64_t)cal_P3)>>8) + ((v1_1*(int64_t)cal_P2)<<12);
	v1_3 = (((((int64_t)1)<<47)+v1_2))*((int64_t)cal_P1)>>33;
	if(v1_3 == 0){
		press[0] = 0;
		return 0; // avoid exception due to divide by zero
	}
	p1 = (int64_t)(1048576-data_int);
	p2 = (((p1<<31)-v2_3)*3125)/v1_3;
	v1_4 = (((int64_t)cal_P9)*(p2>>13)*(p2>>13)) >> 25;
	v2_4 = (((int64_t)cal_P8)*p2) >> 19;
	p3 = ((p2+v1_4+v2_4)>>8) + (((int64_t)cal_P7)<<4);
	uint32_t p4 = (uint32_t) p3;
  	press[0] = (float) (p4/256.0);
	return 0;
}

/**
 * Get Relative humidity measurement from BME280 (in the digitizer)
 *
 * @param data Data string from BME280
 * @param cal Calibration humidity string from BME280
 * @param tf Temperature parameter needed for relative humidity calculation
 * @param relhum Relative humidity measured
 *
 * @return 0
 */
int bme280_get_relhum(char *data,char *cal,int32_t *tf,float *relhum){
	
	char relhum_data[16];
	char substr[5];
	// Get temperature raw data and calibration from BME sampled data
	strncpy(relhum_data,data+12,4);
	relhum_data[4] = '\0';
	// Convert data and calibration
	int32_t data_int;
	unsigned char cal_H1, cal_H3;
	signed char cal_H6;
	signed short cal_H2, cal_H4, cal_H5;
	signed short cal_H41, cal_H42, cal_H51, cal_H52;
	sscanf(relhum_data,"%x",&data_int);
		// Switch endianness

			substr[0] = cal[0]; //0xA1
			substr[1] = cal[1];
			substr[2] = '\0';
			sscanf(substr,"%hhx",&cal_H1);
			substr[2] = cal[2]; //0xE1
			substr[3] = cal[3];
			substr[0] = cal[4]; //0xE2
			substr[1] = cal[5];
			substr[4] = '\0';
			sscanf(substr,"%hx",&cal_H2);
			substr[0] = cal[6]; //0xE3
			substr[1] = cal[7];
			substr[2] = '\0';
			sscanf(substr,"%hhx",&cal_H3);
			substr[0] = cal[8]; //0xE4
			substr[1] = cal[9];
			substr[2] = '\0';
			sscanf(substr,"%hx",&cal_H41);
			substr[0] = cal[10]; //0xE5
			substr[1] = cal[11];
			substr[2] = '\0';
			sscanf(substr,"%hx",&cal_H42);
			cal_H4 = ((cal_H41 << 4)& 0xFF0) + (cal_H42 & 0xF);
			cal_H51 = cal_H42;
			substr[0] = cal[12];  //0xE6
			substr[1] = cal[13];
			substr[2] = '\0';
			sscanf(substr,"%hx",&cal_H52);
			cal_H5 = ((cal_H51 >> 4) & 0x0F) + ((cal_H52 << 4) & 0xFF0);
			substr[0] = cal[14];  //0xE7
			substr[1] = cal[15];
			substr[2] = '\0';
			sscanf(substr,"%hhx",&cal_H6);
	//data_int = data_int>>4;

	// Perform the compensation calculations
	int32_t h1, h2, h3;
	h1 = (tf[0]-((int32_t)76800));
	h2 = (((((data_int<<14)-(((int32_t)cal_H4)<<20)-(((int32_t)cal_H5)*h1)) + ((int32_t)16384)) >> 15) * (((((((h1*((int32_t)cal_H6))>>10) * (((h1*((int32_t)cal_H3))>>11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)cal_H2) + 8192) >> 14));
	h3 = (h2 - (((((h2>>15)*(h2>>15)) >> 7) * ((int32_t)cal_H1)) >> 4));
	h3 = MAX(h3,0);
	h3 = MIN(h3,419430400);

	relhum[0] = (float) (((uint32_t)h3>>12)/1024.0);

	return 0;
}

/** @} */
}