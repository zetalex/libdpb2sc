/************************** Libraries includes *****************************/

#include "dpb2sc.h"

/************************** Semaphore Functions ******************************/
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
}

/************************** Shared Memory Functions ******************************/
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

	memory = shmat(memoryID, NULL, 0);
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
 * Read from Shared memory to get IIO_EVENT_MONITOR Events
 *
 * @param int channel: Channel that triggered the alarm
 * @param char *ev_type: Direction of event triggered
 * @param char *ch_type: Type of channel that triggered the event
 *
 * @return 0
 */
int read_shm(int *channel, char *ev_type, char *ch_type){
	memoryID = shmget(MEMORY_KEY, sizeof(struct wrapper), 0);
	if (memoryID == -1) {
	      perror("Shared memory");
	      return 1;
	   }

	memory = shmat(memoryID, NULL, 0);
	channel[0] = memory->chn;
	strcpy(ev_type,memory->ev_type);
	strcpy(ch_type,memory->ch_type);
	 if (shmdt(memory) == -1) {
	      perror("shmdt");
	      return 1;
	   }
	return 0;
}
/************************** AMS Functions ******************************/

/**
 * Reads temperature of n channels (channels specified in *chan) and stores the values in *res
 *
 * @param int *chan: array which contain channels to measure
 * @param int n: number of channels to measure
 * @param float *res: array where results are stored in
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
			return -1;
			}
		else{
			fseek(raw, 0, SEEK_END);
			long fsize = ftell(raw);
			fseek(raw, 0, SEEK_SET);  /* same as rewind(f); */

			char *raw_string = malloc(fsize + 1);
			fread(raw_string, fsize, 1, raw);

			fseek(offset, 0, SEEK_END);
			fsize = ftell(offset);
			fseek(offset, 0, SEEK_SET);  /* same as rewind(f); */

			char *offset_string = malloc(fsize + 1);
			fread(offset_string, fsize, 1, offset);

			fseek(scale, 0, SEEK_END);
			fsize = ftell(scale);
			fseek(scale, 0, SEEK_SET);  /* same as rewind(f); */

			char *scale_string = malloc(fsize + 1);
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
 * @param int *chan: array which contain channels to measure
 * @param int n: number of channels to measure
 * @param float *res: array where results are stored in
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
			return -1;
			}
		else{

			fseek(raw, 0, SEEK_END);
			long fsize = ftell(raw);
			fseek(raw, 0, SEEK_SET);  /* same as rewind(f); */

			char *raw_string = malloc(fsize + 1);
			fread(raw_string, fsize, 1, raw);

			fseek(scale, 0, SEEK_END);
			fsize = ftell(scale);
			fseek(scale, 0, SEEK_SET);  /* same as rewind(f); */

			char *scale_string = malloc(fsize + 1);
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
 * @param int chan: channel whose alarm limit will be changed
 * @param char *ev_type: string that determines the type of the event
 * @param char *ch_type: string that determines the type of the channel
 * @param float val: value of the new limit
 *
 * @return Negative integer if setting fails, any file could not be opened or invalid argument.If not, returns 0 and the modifies the specified limit
 *
 */
int xlnx_ams_set_limits(int chan, char *ev_type, char *ch_type, float val){
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
					return -EINVAL;
				}
				fseek(offset, 0, SEEK_END);
				fsize = ftell(offset);
				fseek(offset, 0, SEEK_SET);  /* same as rewind(f); */

				char *offset_string = malloc(fsize + 1);
				fread(offset_string, fsize, 1, offset);

				fseek(scale, 0, SEEK_END);
				fsize = ftell(scale);
				fseek(scale, 0, SEEK_SET);  /* same as rewind(f); */

				char *scale_string = malloc(fsize + 1);
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

				char *scale_string = malloc(fsize + 1);
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

/************************** I2C Devices Functions ******************************/
/**
 * Initialize every I2C sensor available
 *
 * @param DPB_I2cSensors *data; struct which contains every I2C sensor available
 *
 * @return 0 and every I2C sensor initialized.
 */
int init_I2cSensors(struct DPB_I2cSensors *data){

	int rc;
	uint64_t timestamp;
	data->dev_pcb_temp.filename = "/dev/i2c-2";
	data->dev_pcb_temp.addr = 0x18;

	data->dev_som_volt.filename = "/dev/i2c-2";
	data->dev_som_volt.addr = 0x40;
	data->dev_sfp0_2_volt.filename = "/dev/i2c-3";
	data->dev_sfp0_2_volt.addr = 0x40;
	data->dev_sfp3_5_volt.filename = "/dev/i2c-3";
	data->dev_sfp3_5_volt.addr = 0x41;

	data->dev_sfp0_A0.filename = "/dev/i2c-6";
	data->dev_sfp0_A0.addr = 0x50;
	data->dev_sfp1_A0.filename = "/dev/i2c-10";
	data->dev_sfp1_A0.addr = 0x50;
	data->dev_sfp2_A0.filename = "/dev/i2c-8";
	data->dev_sfp2_A0.addr = 0x50;
	data->dev_sfp3_A0.filename = "/dev/i2c-12";
	data->dev_sfp3_A0.addr = 0x50;
	data->dev_sfp4_A0.filename = "/dev/i2c-9";
	data->dev_sfp4_A0.addr = 0x50;
	data->dev_sfp5_A0.filename = "/dev/i2c-13";
	data->dev_sfp5_A0.addr = 0x50;

	data->dev_sfp0_A2.filename = "/dev/i2c-6";
	data->dev_sfp0_A2.addr = 0x51;
	data->dev_sfp1_A2.filename = "/dev/i2c-10";
	data->dev_sfp1_A2.addr = 0x51;
	data->dev_sfp2_A2.filename = "/dev/i2c-8";
	data->dev_sfp2_A2.addr = 0x51;
	data->dev_sfp3_A2.filename = "/dev/i2c-12";
	data->dev_sfp3_A2.addr = 0x51;
	data->dev_sfp4_A2.filename = "/dev/i2c-9";
	data->dev_sfp4_A2.addr = 0x51;
	data->dev_sfp5_A2.filename = "/dev/i2c-13";
	data->dev_sfp5_A2.addr = 0x51;

	sem_post(&alarm_sync);
	rc = init_tempSensor(&data->dev_pcb_temp);
	if (rc) {
		timestamp = time(NULL);
		rc = status_alarm_json("DPB","PCB Temperature Sensor I2C Bus Status",99,timestamp,"critical");
	}

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

	rc = init_SFP_A0(&data->dev_sfp0_A0);
	if (rc) {
		timestamp = time(NULL);
		rc = status_alarm_json("DPB","SFP I2C Bus Status",0,timestamp,"critical");
	}
	else{
		rc = init_SFP_A2(&data->dev_sfp0_A2);
		if (rc) {
			timestamp = time(NULL);
			rc = status_alarm_json("DPB","SFP I2C Bus Status",0,timestamp,"critical");
		}
		else{
			sfp0_connected = 1;
		}
	}
	rc = init_SFP_A0(&data->dev_sfp1_A0);
	if (rc) {
		timestamp = time(NULL);
		rc = status_alarm_json("DPB","SFP I2C Bus Status",1,timestamp,"critical");
	}
	else{
		rc = init_SFP_A2(&data->dev_sfp1_A2);
		if (rc) {
			timestamp = time(NULL);
			rc = status_alarm_json("DPB","SFP I2C Bus Status",1,timestamp,"critical");
		}
		else{
			sfp1_connected = 1;
		}
	}
	rc = init_SFP_A0(&data->dev_sfp2_A0);
	if (rc) {
		timestamp = time(NULL);
		rc = status_alarm_json("DPB","SFP I2C Bus Status",2,timestamp,"critical");
	}
	else{
		rc = init_SFP_A2(&data->dev_sfp2_A2);
		if (rc) {
			timestamp = time(NULL);
			rc = status_alarm_json("DPB","SFP I2C Bus Status",2,timestamp,"critical");
		}
		else{
			sfp2_connected = 1;
		}
	}
	rc = init_SFP_A0(&data->dev_sfp3_A0);
	if (rc) {
		timestamp = time(NULL);
		rc = status_alarm_json("DPB","SFP I2C Bus Status",3,timestamp,"critical");
	}
	else{
		rc = init_SFP_A2(&data->dev_sfp3_A2);
		if (rc) {
			timestamp = time(NULL);
			rc = status_alarm_json("DPB","SFP I2C Bus Status",3,timestamp,"critical");
		}
		else{
			sfp3_connected = 1;
		}
	}
	rc = init_SFP_A0(&data->dev_sfp4_A0);
	if (rc) {
		timestamp = time(NULL);
		rc = status_alarm_json("DPB","SFP I2C Bus Status",4,timestamp,"critical");
	}
	else{
		rc = init_SFP_A2(&data->dev_sfp4_A2);
		if (rc) {
			timestamp = time(NULL);
			rc = status_alarm_json("DPB","SFP I2C Bus Status",4,timestamp,"critical");
		}
		else{
			sfp4_connected = 1;
		}
	}
	rc = init_SFP_A0(&data->dev_sfp5_A0);
	if (rc) {
		timestamp = time(NULL);
		rc = status_alarm_json("DPB","SFP I2C Bus Status",5,timestamp,"critical");
	}
	else{
		rc = init_SFP_A2(&data->dev_sfp5_A2);
		if (rc) {
			timestamp = time(NULL);
			rc = status_alarm_json("DPB","SFP I2C Bus Status",5,timestamp,"critical");
		}
		else{
			sfp5_connected = 1;
		}
	}
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
 * @param DPB_I2cSensors *data: struct which contains every I2C sensor available
 *
 * @returns 0.
 */
int stop_I2cSensors(struct DPB_I2cSensors *data){

	i2c_stop(&data->dev_pcb_temp);

	i2c_stop(&data->dev_sfp0_2_volt);
	i2c_stop(&data->dev_sfp3_5_volt);
	i2c_stop(&data->dev_som_volt);

	i2c_stop(&data->dev_sfp0_A0);
	i2c_stop(&data->dev_sfp1_A0);
	i2c_stop(&data->dev_sfp2_A0);
	i2c_stop(&data->dev_sfp3_A0);
	i2c_stop(&data->dev_sfp4_A0);
	i2c_stop(&data->dev_sfp5_A0);

	i2c_stop(&data->dev_sfp0_A2);
	i2c_stop(&data->dev_sfp1_A2);
	i2c_stop(&data->dev_sfp2_A2);
	i2c_stop(&data->dev_sfp3_A2);
	i2c_stop(&data->dev_sfp4_A2);
	i2c_stop(&data->dev_sfp5_A2);

	return 0;
}

/************************** Temp.Sensor Functions ******************************/
/**
 * Initialize MCP9844 Temperature Sensor
 *
 * @param I2cDevice *dev: device to be initialized
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
 * @param struct DPB_I2cSensors *data: being the corresponding I2C device for the MCP9844 Temperature Sensor
 * @param float *res: where the ambient temperature value is stored
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
 * @param struct DPB_I2cSensors *data: being the corresponding I2C device for the MCP9844 Temperature Sensor
 * @param int n: which limit is modified
 * @param short temp: value of the limit that is to be set
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
 * @param struct DPB_I2cSensors *data: being the corresponding I2C device for the MCP9844 Temperature Sensor
 * @param uint8_t *bit_ena: array which should contain the desired bit value (0 o 1)
 * @param uint8_t *bit_num: array which should contain the position of the bit/s that will be modified
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
 * @param uint8_t flag_buf: contains alarm flags
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
 * @param struct DPB_I2cSensors *data: being the corresponding I2C device for the MCP9844 Temperature Sensor
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

/************************** SFP Functions ******************************/
/**
 * Initialize SFP EEPROM page 1 as an I2C device
 *
 * @param I2cDevice *dev: SFP of which EEPROM is to be initialized
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
 * @param I2cDevice *dev: SFP of which EEPROM is to be initialized
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
 * @param I2cDevice *dev: SFP of which the checksum is to be checked
 * @param uint8_t ini_reg: Register where the checksum count starts
 * @param int size: number of registers summed for the checksum
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
 * @param struct DPB_I2cSensors *data: I2C devices
 * @param int n: indicate from which of the 6 SFP is going to be read
 * @param float *res where the magnitude value is stored
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

	switch(n){
		case DEV_SFP0:
			if(sfp0_connected){
				dev = data->dev_sfp0_A2;
			}
			else{
				return -EINVAL;
			}
		break;
		case DEV_SFP1:
			if(sfp1_connected){
				dev = data->dev_sfp1_A2;
			}
			else{
				return -EINVAL;
			}
		break;
		case DEV_SFP2:
			if(sfp2_connected){
				dev = data->dev_sfp2_A2;
			}
			else{
				return -EINVAL;
			}
		break;
		case DEV_SFP3:
				if(sfp3_connected){
				dev = data->dev_sfp3_A2;
			}
			else{
				return -EINVAL;
			}
			break;
		case DEV_SFP4:
			if(sfp4_connected){
				dev = data->dev_sfp4_A2;
			}
			else{
				return -EINVAL;
			}
			break;
		case DEV_SFP5:
			if(sfp5_connected){
				dev = data->dev_sfp5_A2;
			}
			else{
				return -EINVAL;
			}
			break;
		default:
			return -EINVAL;
		break;
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
 * @param struct DPB_I2cSensors *data: I2C devices
 * @param int n: indicate from which of the 6 SFP is going to be read
 * @param float *res: where the magnitude value is stored
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

	switch(n){
		case DEV_SFP0:
			if(sfp0_connected){
				dev = data->dev_sfp0_A2;
			}
			else{
				return -EINVAL;
			}
		break;
		case DEV_SFP1:
			if(sfp1_connected){
				dev = data->dev_sfp1_A2;
			}
			else{
				return -EINVAL;
			}
		break;
		case DEV_SFP2:
			if(sfp2_connected){
				dev = data->dev_sfp2_A2;
			}
			else{
				return -EINVAL;
			}
		break;
		case DEV_SFP3:
				if(sfp3_connected){
				dev = data->dev_sfp3_A2;
			}
			else{
				return -EINVAL;
			}
			break;
		case DEV_SFP4:
			if(sfp4_connected){
				dev = data->dev_sfp4_A2;
			}
			else{
				return -EINVAL;
			}
			break;
		case DEV_SFP5:
			if(sfp5_connected){
				dev = data->dev_sfp5_A2;
			}
			else{
				return -EINVAL;
			}
			break;
		default:
			return -EINVAL;
		break;
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
 * @param struct DPB_I2cSensors *data: I2C devices
 * @param int n: indicate from which of the 6 SFP is going to be read
 * @param float *res: where the magnitude value is stored
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

	switch(n){
		case DEV_SFP0:
			if(sfp0_connected){
				dev = data->dev_sfp0_A2;
			}
			else{
				return -EINVAL;
			}
		break;
		case DEV_SFP1:
			if(sfp1_connected){
				dev = data->dev_sfp1_A2;
			}
			else{
				return -EINVAL;
			}
		break;
		case DEV_SFP2:
			if(sfp2_connected){
				dev = data->dev_sfp2_A2;
			}
			else{
				return -EINVAL;
			}
		break;
		case DEV_SFP3:
				if(sfp3_connected){
				dev = data->dev_sfp3_A2;
			}
			else{
				return -EINVAL;
			}
			break;
		case DEV_SFP4:
			if(sfp4_connected){
				dev = data->dev_sfp4_A2;
			}
			else{
				return -EINVAL;
			}
			break;
		case DEV_SFP5:
			if(sfp5_connected){
				dev = data->dev_sfp5_A2;
			}
			else{
				return -EINVAL;
			}
			break;
		default:
			return -EINVAL;
		break;
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
 * @param struct DPB_I2cSensors *data: I2C devices
 * @param int n: indicate from which of the 6 SFP is going to be read
 * @param float *res: where the magnitude value is stored
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

	switch(n){
		case DEV_SFP0:
			if(sfp0_connected){
				dev = data->dev_sfp0_A2;
			}
			else{
				return -EINVAL;
			}
		break;
		case DEV_SFP1:
			if(sfp1_connected){
				dev = data->dev_sfp1_A2;
			}
			else{
				return -EINVAL;
			}
		break;
		case DEV_SFP2:
			if(sfp2_connected){
				dev = data->dev_sfp2_A2;
			}
			else{
				return -EINVAL;
			}
		break;
		case DEV_SFP3:
				if(sfp3_connected){
				dev = data->dev_sfp3_A2;
			}
			else{
				return -EINVAL;
			}
			break;
		case DEV_SFP4:
			if(sfp4_connected){
				dev = data->dev_sfp4_A2;
			}
			else{
				return -EINVAL;
			}
			break;
		case DEV_SFP5:
			if(sfp5_connected){
				dev = data->dev_sfp5_A2;
			}
			else{
				return -EINVAL;
			}
			break;
		default:
			return -EINVAL;
		break;
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
 * @param struct DPB_I2cSensors *data: I2C devices
 * @param int n: indicate from which of the 6 SFP is going to be read,
 * @param float *res: where the magnitude value is stored
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

	switch(n){
		case DEV_SFP0:
			if(sfp0_connected){
				dev = data->dev_sfp0_A2;
			}
			else{
				return -EINVAL;
			}
		break;
		case DEV_SFP1:
			if(sfp1_connected){
				dev = data->dev_sfp1_A2;
			}
			else{
				return -EINVAL;
			}
		break;
		case DEV_SFP2:
			if(sfp2_connected){
				dev = data->dev_sfp2_A2;
			}
			else{
				return -EINVAL;
			}
		break;
		case DEV_SFP3:
				if(sfp3_connected){
				dev = data->dev_sfp3_A2;
			}
			else{
				return -EINVAL;
			}
			break;
		case DEV_SFP4:
			if(sfp4_connected){
				dev = data->dev_sfp4_A2;
			}
			else{
				return -EINVAL;
			}
			break;
		case DEV_SFP5:
			if(sfp5_connected){
				dev = data->dev_sfp5_A2;
			}
			else{
				return -EINVAL;
			}
			break;
		default:
			return -EINVAL;
		break;
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
 * @param struct DPB_I2cSensors *data: I2C devices
 * @param int n: indicate from which of the 6 SFP is dealing with
 * @param uint8_t * res : stores the current RX_LOS and TX_FAULT status
 *
 * @return 0 if reads properly and stores 0 or 1 depending on the current states (1 if status asserted, 0 if not)
 */
int sfp_avago_read_status(struct DPB_I2cSensors *data,int n,uint8_t *res) {
	int rc = 0;
	uint8_t status_buf[1] = {0};
	uint8_t status_reg = SFP_STAT_REG;

	struct I2cDevice dev;

	switch(n){
		case DEV_SFP0:
			if(sfp0_connected){
				dev = data->dev_sfp0_A2;
			}
			else{
				return -EINVAL;
			}
		break;
		case DEV_SFP1:
			if(sfp1_connected){
				dev = data->dev_sfp1_A2;
			}
			else{
				return -EINVAL;
			}
		break;
		case DEV_SFP2:
			if(sfp2_connected){
				dev = data->dev_sfp2_A2;
			}
			else{
				return -EINVAL;
			}
		break;
		case DEV_SFP3:
				if(sfp3_connected){
				dev = data->dev_sfp3_A2;
			}
			else{
				return -EINVAL;
			}
			break;
		case DEV_SFP4:
			if(sfp4_connected){
				dev = data->dev_sfp4_A2;
			}
			else{
				return -EINVAL;
			}
			break;
		case DEV_SFP5:
			if(sfp5_connected){
				dev = data->dev_sfp5_A2;
			}
			else{
				return -EINVAL;
			}
			break;
		default:
			return -EINVAL;
		break;
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
 * @param uint16_t flags: contains alarms flags
 * @param int n: indicate from which of the 6 SFP is dealing with
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
 * @param uint16_t flags: contains alarms flags
 * @param int n: indicate from which of the 6 SFP is dealing with
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
 * @param struct DPB_I2cSensors *data: being the corresponding I2C device for the SFP EEPROM page 2
 * @param int n: indicate from which of the 6 SFP is going to be read
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

	switch(n){
		case DEV_SFP0:
			if(sfp0_connected){
				dev = data->dev_sfp0_A2;
			}
			else{
				return -EINVAL;
			}
		break;
		case DEV_SFP1:
			if(sfp1_connected){
				dev = data->dev_sfp1_A2;
			}
			else{
				return -EINVAL;
			}
		break;
		case DEV_SFP2:
			if(sfp2_connected){
				dev = data->dev_sfp2_A2;
			}
			else{
				return -EINVAL;
			}
		break;
		case DEV_SFP3:
				if(sfp3_connected){
				dev = data->dev_sfp3_A2;
			}
			else{
				return -EINVAL;
			}
			break;
		case DEV_SFP4:
			if(sfp4_connected){
				dev = data->dev_sfp4_A2;
			}
			else{
				return -EINVAL;
			}
			break;
		case DEV_SFP5:
			if(sfp5_connected){
				dev = data->dev_sfp5_A2;
			}
			else{
				return -EINVAL;
			}
			break;
		default:
			return -EINVAL;
		break;
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

/************************** Volt. and Curr. Sensor Functions ******************************/
/**
 * Initialize INA3221 Voltage and Current Sensor
 *
 * @param I2cDevice *dev: device to be initialized
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
 * @param struct DPB_I2cSensors *data: being the corresponding I2C device INA3221 Voltage and Current Sensor
 * @param int n: indicate from which of the 3 INA3221 is going to be read,float *res where the voltage values are stored
 * @param float *res: storage of collected data
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
 * @param struct DPB_I2cSensors *data: being the corresponding I2C device INA3221 Voltage and Current Sensor
 * @param int n: indicate from which of the 3 INA3221 is going to be read,float *res where the current values are stored
 * @param float *res: storage of collected data
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
 * @param struct DPB_I2cSensors *data: being the corresponding I2C device INA3221 Voltage and Current Sensor
 * @param uint16_t mask: contains critical alarm flags
 * @param int n: indicate from which of the 3 INA3221 is dealing with
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
 * @param struct DPB_I2cSensors *data: being the corresponding I2C device INA3221 Voltage and Current Sensor
 * @param uint16_t mask: contains warning alarm flags
 * @param int n: indicate from which of the 3 INA3221 is dealing with
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
 * @param struct DPB_I2cSensors *data: being the corresponding I2C device for the INA3221 Voltage and Current Sensor
 * @param int n: indicate from which of the 3 INA3221 is going to be read
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
 * @param struct DPB_I2cSensors *data: being the corresponding I2C device for the MCP9844 Temperature Sensor
 * @param int n: which of the 3 INA3221 is being dealt with
 * @param int ch: which of the 3 INA3221 channels is being dealt with
 * @param int alarm_type: indicates if the limit to be modifies is for a critical alarm or warning alarm
 * @param float curr: current value which will be the new limit
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
 * @param struct DPB_I2cSensors *data: being the corresponding I2C device for the INA3221 Voltage Sensor
 * @param uint8_t *bit_ena: array which should contain the desired bit value (0 o 1)
 * @param uint8_t *bit_num: array which should contain the position of the bit/s that will be modified
 * @param int n :which of the 3 INA3221 is being dealt with
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
/************************** JSON functions ******************************/
/**
 * Parses monitoring data into a JSON array so as to include it in a JSON object
 *
 * @param json_object *jarray: JSON array in which the data will be stored
 * @param int chan: Number of measured channel, if chan is 99 means channel will not be parsed
 * @param float val: Measured magnitude value
 * @param char *magnitude: Name of the measured magnitude
 *
 * @return 0
 */
int parsing_mon_sensor_data_into_array (json_object *jarray,float val, char *magnitude, int chan)
{
	struct json_object *jobj,*jstring,*jint,*jdouble = NULL;
	jobj = json_object_new_object();
	char buffer[32];

	sprintf(buffer, "%3.4f", val);
	jdouble = json_object_new_double_s((double) val,buffer);
	jstring = json_object_new_string(magnitude);

	json_object_object_add(jobj,"magnitudename", jstring);
	if (chan != 99){
		jint = json_object_new_int(chan);
		json_object_object_add(jobj,"channel", jint);
	}
	json_object_object_add(jobj,"value", jdouble);

	json_object_array_add(jarray,jobj);
	return 0;
}

/**
 * Parses monitoring string data into a JSON array so as to include it in a JSON object
 *
 * @param json_object *jarray: JSON array in which the data will be stored
 * @param int chan: Number of measured channel, if chan is 99 means channel will not be parsed
 * @param char val: Measured magnitude value in string format
 * @param char *magnitude: Name of the measured magnitude
 *
 * @return 0
 */
int parsing_mon_sensor_string_into_array(json_object *jarray,char *val, char *magnitude, int chan)
{
	struct json_object *jobj,*jstring,*jint,*jstring_val = NULL;
	jobj = json_object_new_object();
	char buffer[8];

	jstring_val = json_object_new_string(val);
	jstring = json_object_new_string(magnitude);

	json_object_object_add(jobj,"magnitudename", jstring);
	if (chan != 99){
		jint = json_object_new_int(chan);
		json_object_object_add(jobj,"channel", jint);
	}
	json_object_object_add(jobj,"value", jstring_val);

	json_object_array_add(jarray,jobj);
	return 0;
}

/**
 * Parses monitoring status data into a JSON array so as to include it in a JSON object
 *
 * @param json_object *jarray: JSON array in which the data will be stored
 * @param int status: Value of the status
 * @param char *magnitude: Name of the measured magnitude/interface
 * @param int chan: Number of measured channel, if chan is 99 means channel will not be parsed
 *
 * @return 0
 */
int parsing_mon_status_data_into_array(json_object *jarray, int status, char *magnitude, int chan)
{
	struct json_object *jobj,*jstring,*jint,*jstatus = NULL;
	jobj = json_object_new_object();

	jstring = json_object_new_string(magnitude);
	if(status == 1)
		jstatus = json_object_new_string("ON");
	else if (status == 0)
		jstatus = json_object_new_string("OFF");

	json_object_object_add(jobj,"magnitudename", jstring);
	if (chan != 99){
		jint = json_object_new_int(chan);
		json_object_object_add(jobj,"channel", jint);
	}
	json_object_object_add(jobj,"value", jstatus);

	json_object_array_add(jarray,jobj);
	return 0;
}
/**
 * Parses alarms data into a JSON string and send it to socket
 *

 * @param int chan: Number of measured channel, if chan is 99 means channel will not be parsed
 * @param float val: Measured magnitude value
 * @param char *board: Board that triggered the alarm
 * @param char *chip: Name of the chip that triggered the alarm
 * @param char *ev_type: Type of event that has occurred
 * @param uint64_t timestamp: Time when the event occurred
 * @param char *info_type: Determines the reported event type (info: warning or critical)
 *
 *
 * @return 0 or negative integer if validation fails
 */
int alarm_json (char *board,char *chip,char *ev_type, int chan, float val,uint64_t timestamp,char *info_type)
{
	sem_wait(&alarm_sync);
	struct json_object *jalarm_data,*jboard,*jchip,*jtimestamp,*jchan,*jdouble,*jev_type = NULL;
	jalarm_data = json_object_new_object();
	char buffer[8];
	uint8_t level = 1;

	uint64_t timestamp_msg = time(NULL)*1000;

	sprintf(buffer, "%3.4f", val);

	char *device = "ID DPB";
	jboard = json_object_new_string(board);
	if(!strcmp(info_type,"critical")){
		level = 0;
	}
	else if(!strcmp(info_type,"warning")){
		level = 1;
	}
	else{
		return -EINVAL;
	}

	json_object_object_add(jalarm_data,"board", jboard);

	jdouble = json_object_new_double_s((double) val,buffer);
	jchip = json_object_new_string(chip);
	jev_type = json_object_new_string(ev_type);
	jtimestamp = json_object_new_int64(timestamp*1000);

	json_object_object_add(jalarm_data,"magnitudename", jchip);
	json_object_object_add(jalarm_data,"eventtype", jev_type);
	json_object_object_add(jalarm_data,"eventtimestamp", jtimestamp);
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
 * @param int chan: Number of measured channel, if chan is 99 means channel will not be parsed (also indicates it is not SFP related)
 * @param char *chip: Name of the chip that triggered the alarm
 * @param char *board: Name of the board where the alarm is asserted
 * @param uint64_t timestamp: Time when the event occurred
 * @param char *info_type: Determines the reported event type (info,warning or critical)
 *
 *
 * @return 0 or negative integer if validation fails
 */
int status_alarm_json (char *board,char *chip, int chan,uint64_t timestamp,char *info_type)
{
	sem_wait(&alarm_sync);
	struct json_object *jalarm_data,*jboard,*jchip,*jtimestamp,*jchan,*jstatus = NULL;
	jalarm_data = json_object_new_object();

	uint64_t timestamp_msg = (time(NULL))*1000;
	uint8_t level = 1;
	char *device = "ID DPB";

	jboard = json_object_new_string(board);

	json_object_object_add(jalarm_data,"board", jboard);
	if(!strcmp(info_type,"critical")){
		level = 0;
	}
	else{
		return -EINVAL;
	}

	jchip = json_object_new_string(chip);
	jtimestamp = json_object_new_int64(timestamp_msg);

	json_object_object_add(jalarm_data,"magnitudename", jchip);
	json_object_object_add(jalarm_data,"eventtimestamp", jtimestamp);
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
 * Parses command response into a JSON string and send it to socket
 *
 * @param int msg_id: Message ID
 * @param float val: read value
 * @param char* cmd_reply: Stores CMD JSON reply to send it
 *
 * @return 0 or negative integer if validation fails
 */
int command_response_json (int msg_id, float val, char* cmd_reply)
{
	json_object *jcmd_data = json_object_new_object();

	char buffer[8];
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

	sprintf(buffer, "%3.4f", val);
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
 * @param int msg_id: Message ID
 * @param int val: read value (1 is ON and 0 is OFF), if operation is set val = 99, JSON value field = OK , else is error, JSON value = ERROR
 * @param char* cmd_reply: Stores CMD JSON reply to send it
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

	printf("COMANDO AQUI 2 \n");
	sleep(1);

	if(val == 99)
		jval = json_object_new_string("OK");
	else if(val == 0)
		jval = json_object_new_string("ON");
	else if(val == 1)
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
	const char *serialized_json1 = json_object_to_json_string(jmsg_id);
	const char *serialized_json2 = json_object_to_json_string(jmsg_time);
	const char *serialized_json3 = json_object_to_json_string(jmsg_type);
	const char *serialized_json4 = json_object_to_json_string(jval);
	const char *serialized_json5 = json_object_to_json_string(juuid);
	printf("%s \n",serialized_json1);
	printf("%s \n",serialized_json2);
	printf("%s \n",serialized_json3);
	printf("%s \n",serialized_json4);
	printf("%s \n",serialized_json5);
	printf("COMANDO AQUI 3 \n");
	sleep(1);
	const char *serialized_json = json_object_to_json_string(jcmd_data);
	printf("%s \n",serialized_json);
	int rc = json_schema_validate("JSONSchemaSlowControl.json",serialized_json, "cmd_temp.json");
	if (rc) {
		printf("Error\r\n");
		return rc;
	}
	printf("COMANDO AQUI 4 \n");
	sleep(2);
	strcpy(cmd_reply,serialized_json);
	//zmq_send(cmd_router, serialized_json, strlen(serialized_json), 0);
	json_object_put(jcmd_data);
	printf("COMANDO AQUI 5 \n");
	sleep(1);
	return 0;
}
/**
 * Validates generated JSON string with a validation schema
 *
 * @param char *schema: Name of validation schema file
 * @param const char *json_string: JSON string to be validated
 * @param char *temp_file: Name of Temporal File
 *
 * @return 0 if correct, negative integer if validation failed
 */
int json_schema_validate (char *schema,const char *json_string, char *temp_file)
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

/************************** GPIO functions ******************************/
/**
 * Gets GPIO base address
 *
 * @param int *address: pointer where the read GPIO base address plus corresponding offset will be stored
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

		    	char *add_string = malloc(fsize + 1);
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
 * @param int address: GPIO address where the value is going to be written
 * @param int value: value which will be written (0 o 1)
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
    static char *dir = "out";

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
 * @param int address: GPIO address where the desired value is stored
 * @param int *value: pointer where the read value will be stored
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
    static char *dir = "in";
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

	char *value_string = malloc(fsize + 1);
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
 * @return NULL
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
/************************** External monitoring (via GPIO) functions ******************************/
/**
 * Checks from GPIO if Ethernet Links status and reports it
 *
 * @param char *eth_interface: Name of the Ethernet interface
 * @param int status: value of the Ethernet interface status
 *
 * @return  0 if parameters are OK, if not negative integer
 */
int eth_link_status (char *eth_interface, int *status)
{
	int rc = 0;
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
		return -EINVAL;
	}
	return 0;

}

/************************** External monitoring (via GPIO) functions ******************************/
/**
 * Updates Ethernet interface status to ON/OFF
 *
 * @param char *eth_interface: Name of the Ethernet interface
 * @param int val: value of the Ethernet interface status
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
/************************** External alarms (via GPIO) functions ******************************/

/**
* Checks from GPIO if Ethernet Links status has changed from up to down and reports it if necessary
*
* @param char *str: Name of the Ethernet interface
* @param int flag: value of the Ethernet interface flag, determines if the link was previously up
*
* @return  0 if parameters OK and reports the event, if not returns negative integer.
*/
int eth_down_alarm(char *str,int *flag){

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
 * @param int aurora_link: Choose main or backup link of Dig0 or Dig1 (O: Dig0 Main, 1:Dig0 Backup, 2:Dig1 Main, 3:Dig1 Backup)
 * @param int flags: indicates current status of the link
 *
 * @return  0 if parameters are OK, if not negative integer
 */
int aurora_down_alarm(int aurora_link,int *flag){

	int aurora_status[1];
	int rc = 0;
	int address = 0;
	uint64_t timestamp ;
	char *link_id;

    if((flag[0] != 0) && (flag[0] != 1)){
    	return -EINVAL;
    }
	if((aurora_link>3) | (aurora_link<0)){
		return -EINVAL;}
	switch(aurora_link){
	case 0:
		address = DIG0_MAIN_AURORA_LINK;
		link_id = "Aurora Main Link Status";
		break;
	case 1:
		address = DIG0_BACKUP_AURORA_LINK;
		link_id = "Aurora Backup Link Status";
		break;
	case 2:
		address = DIG1_MAIN_AURORA_LINK;
		link_id = "Aurora Main Link Status";
		break;
	case 3:
		address = DIG1_BACKUP_AURORA_LINK;
		link_id = "Aurora Backup Link Status";
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
			rc = status_alarm_json("Dig0",link_id,99,timestamp,"critical");
			return rc;
		}
		else{
			timestamp = time(NULL);
			rc = status_alarm_json("Dig1",link_id,99,timestamp,"critical");
			return rc;
		}
	}
	return 0;
}

/************************** ZMQ Functions******************************/

/**
 * Initializes ZMQ monitoring, command and alarms sockets
 *
 *
 * @return 0 if parameters OK and reports the event. If not returns negative integer.
 */
int zmq_socket_init (){

	int rc = 0;
	int linger = 0;
	int sndhwm = 1;
	size_t sndhwm_size = sizeof(sndhwm);
	size_t linger_size = sizeof(linger);

    zmq_context = zmq_ctx_new();
    mon_publisher = zmq_socket(zmq_context, ZMQ_PUB);

    zmq_setsockopt(mon_publisher, ZMQ_SNDHWM, &sndhwm, sndhwm_size);
    zmq_setsockopt(mon_publisher, ZMQ_RCVHWM, &sndhwm, sndhwm_size);
    zmq_setsockopt (mon_publisher, ZMQ_LINGER, &linger, linger_size);
    rc = zmq_bind(mon_publisher, "tcp://*:5555");
	if (rc) {
		return rc;
	}

    alarm_publisher = zmq_socket(zmq_context, ZMQ_PUB);
    zmq_setsockopt(alarm_publisher, ZMQ_SNDHWM, &sndhwm, sndhwm_size);
    zmq_setsockopt(alarm_publisher, ZMQ_RCVHWM, &sndhwm, sndhwm_size);
    zmq_setsockopt (alarm_publisher, ZMQ_LINGER, &linger, linger_size);
    rc = zmq_bind(alarm_publisher, "tcp://*:5556");
	if (rc) {
		return rc;
	}

    cmd_router = zmq_socket(zmq_context, ZMQ_REP);
    rc = zmq_bind(cmd_router, "tcp://*:5557");
    zmq_setsockopt(cmd_router, ZMQ_SNDHWM, &sndhwm, sndhwm_size);
    zmq_setsockopt(cmd_router, ZMQ_RCVHWM, &sndhwm, sndhwm_size);
    zmq_setsockopt (cmd_router, ZMQ_LINGER, &linger, linger_size);
	if (rc) {
		return rc;
	}
	return 0;
}

/************************** Hash Tables Functions ******************************/
int populate_hv_hash_table(int table_size, char **keys, char **values) {
	struct cmd_uthash *s = NULL; 
	for(int i = 0 ; i < table_size ; i++){
		s = (struct cmd_uthash *) malloc(sizeof *s);
		strcpy(s->daq_word, keys[i]);
    	strcpy(s->board_word, values[i]);
		HASH_ADD_STR(hv_cmd_table, daq_word, s);  /* id: name of key field */
	}
	free(s);
	return 0;
}

int populate_lv_hash_table(int table_size, char **keys, char **values) {
	struct cmd_uthash *s = NULL;

	for(int i = 0 ; i < table_size ; i++){
		s = (struct cmd_uthash *) malloc(sizeof *s);
		strcpy(s->daq_word, keys[i]);
    	strcpy(s->board_word, values[i]);
		HASH_ADD_STR(lv_cmd_table, daq_word, s);  /* id: name of key field */
	}
	free(s);
	return 0;
}

int get_hv_hash_table_command(char *key, char *value) {
	printf("Geteando hash table HV \n");
	struct cmd_uthash *s;
	HASH_FIND_STR(hv_cmd_table,key,s);
	if(s)
		printf("Perfecto geteado en LV\n");
	strcpy(value,s->board_word);
	return 0;
}

int get_lv_hash_table_command(char *key, char *value) {
	printf("Geteando hash table LV \n");
	struct cmd_uthash *s;
	HASH_FIND_STR(lv_cmd_table,key,s);
	if(s)
		printf("Perfecto geteado en LV\n");
	strcpy(value,s->board_word);
	return 0;
}

/************************** Command handling Functions******************************/

/**
* Handles received DPB command
*
* @param DPB_I2cSensors *data: Struct that contains I2C devices
* @param char **cmd: Segmented command
* @param int msg_id: Unique identifier of the received JSON command request message
* @param char *cmd_reply: Stores command JSON reply to send it
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

int dig_command_handling(char **cmd){
	int rc = 0;
	return rc;
}

/**
 * Transforms DPB style command to a CAEN formatted command for HV/LV.
 *
 * @param char *hvlvcmd: Beginning of the command string for CAEN command, to distinguish between HV/LV
 * Should be "$BD:0/1,$CMD:"
 * @param const char *cmd: valid DPB formatted command split into words
 * @param char *result: number of words of the DPB formatted command
 *
 * @return 0 if correct, -ETIMEDOUT if no answer is received after several retries
 */
int hv_lv_command_translation(char *hvlvcmd, char **cmd, int words_n){
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
	printf("Paso 3\n");
	strcat(hvlvcmd,"PAR:");
	char opcode[8];
	printf("Paso 4\n");
	if(!strcmp(cmd[1],"LV")){
		printf("Paso 4.25\n");
		get_lv_hash_table_command(cmd[2],opcode);
	}
	else{
		if(!strcmp(cmd[0],"SET") && !strcmp(cmd[2],"VOLT") ){
			strcpy(opcode, "VSET");
		}
		else if(!strcmp(cmd[0],"SET") && !strcmp(cmd[2],"CURR") ){
			strcpy(opcode, "ISET");
		}
		else {
			printf("Paso 4.5\n");
			get_hv_hash_table_command(cmd[2],opcode);
		}
	}
	strcat(hvlvcmd,opcode);
	printf("Paso 5\n");
	if(words_n==5){
		strcat(hvlvcmd,",VAL:");
		strcat(hvlvcmd,cmd[4]);
	}
	strcat(hvlvcmd,"\r\n");
	return 0;

}

/**
 * Takes a CAEN formatted command for HV/LV and sends it through serial ports
 * Then it awaits for an answer, with a given timeout.
 *
 * @param char *board_dev: file location of the serial port connected to HV/LV 
 * @param const char *cmd: valid CAEN formatted command
 * @param char *result: result of the command
 *
 * @return 0 if correct, -ETIMEDOUT if no answer is received after several retries
 */

int hv_lv_command_handling(char *board_dev, char *cmd, char *result){
	int serial_port_UL3;
	int n;
	struct termios tty;
	char read_buf[128];
	char temp_buf[128];
	// Try with UL3
	for(int i = 0 ; i < SERIAL_PORT_RETRIES ; i++){
		//Open one device (UL3)
		serial_port_UL3 = open(board_dev,O_RDWR);
		// Wait until acquiring non-blocking exclusive lock
    	while(flock(serial_port_UL3, LOCK_EX | LOCK_NB) == -1) {
			usleep(5000);
    	}
		setup_serial_port(serial_port_UL3);
		if (serial_port_UL3 < 0) {
			//Send alarm
			status_alarm_json("HV/LV","UART Lite 3", 99,0,"warning");
			return -EACCES;
		}
		write(serial_port_UL3, cmd, strlen(cmd));
		usleep(10000);
		// Keep reading until timeout (VTIME)
		n = read(serial_port_UL3, temp_buf, sizeof(temp_buf));
		strcat(read_buf,temp_buf);
		close(serial_port_UL3);
		if(temp_buf[n-1] != '\n'){	//Check for LF
			//Send Warning
			status_alarm_json("HV/LV","UART Lite 3", 99,0,"warning");
			count_fails_until_success++;
			count_since_reset++;
		}
		else{
			count_fails_until_success = 0;
			strcpy(result,read_buf);
			goto success;
		}
	}
	//Send Critical error
	status_alarm_json("HV/LV","UART Lite 3", 99,0,"critical");
	flock(serial_port_UL3, LOCK_UN); 
	return -ETIMEDOUT;
success:	
	flock(serial_port_UL3, LOCK_UN); 
	return 0;
}

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
	return 0;
}

/************************** Exit function declaration ******************************/
/**
 * Closes ZMQ sockets and GPIOs when exiting.
 */
/*void atexit_function() {
	unexport_GPIO();
    zmq_close(mon_publisher);
    zmq_close(alarm_publisher);
    zmq_close(cmd_router);
}*/
/************************** Signal Handling function declaration ******************************/
/**
 * Handles library closing, closing zmq context and removing sysfs GPIO folders
 *
 * @param void
 *
 * @return void
 */
void lib_close() {
   unexport_GPIO();
   zmq_close(mon_publisher);
   zmq_close(alarm_publisher);
   zmq_close(cmd_router);
   zmq_ctx_shutdown(zmq_context);
   zmq_ctx_destroy(zmq_context);

   break_flag = 1;
   return;
}

/************************** UUID generator function declaration ******************************/
/**
 * Generates UUID
 *
 * @param char *uuid: String where UUID is stored
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

