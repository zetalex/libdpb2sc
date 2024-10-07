#ifndef __DPB2SC_H_INCLUDED__
#define __DPB2SC_H_INCLUDED__

extern "C" {
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include "timer.h"
#include <semaphore.h>
#include <sys/shm.h>
#include <sys/file.h>
#include <fcntl.h>
#include <time.h>
#include <zmq.h>
#include "json-c/json.h"
#include <math.h>
#include <dirent.h>
#include <signal.h>
#include <regex.h>
#include <termios.h>
#include "uthash.h"
#include <linux/serial.h>
#include <sys/ioctl.h>

#include "i2c.h"
#include "linux/errno.h"
#include <COPacketCmdHkDig.h>

/************************** Main Struct Definition *****************************/

struct DPB_I2cSensors{

	struct I2cDevice dev_pcb_temp;
	struct I2cDevice dev_sfp0_2_volt;
	struct I2cDevice dev_sfp3_5_volt;
	struct I2cDevice dev_som_volt;
    struct I2cDevice dev_sfp_A0[6];
    struct I2cDevice dev_sfp_A2[6];
};
/******************************************************************************
*Local Semaphores.
****************************************************************************/
/** @defgroup semaph Local Semaphores
 *  Semaphores needed to synchronize the application execution and avoid race conditions
 *  @{
 */

/** @brief Semaphore to synchronize I2C Bus usage */
sem_t i2c_sync;

/** @brief Semaphore to synchronize GPIO file accesses */
sem_t file_sync;

/** @brief Semaphore to synchronize GPIO AND Ethernet file accesses */
sem_t alarm_sync;

/** @brief Semaphore to avoid race conditions when JSON validating */
sem_t sem_valid;

/** @brief Semaphore to avoid race conditions when reading from HV/LV serial port */
sem_t sem_hvlv;

/** @brief Semaphore to avoid race conditions when reading from Digitizer 0 serial port */
sem_t sem_dig0;

/** @brief Semaphore to avoid race conditions when reading from Digitizer 1 serial port */
sem_t sem_dig1;

/** @} */

/******************************************************************************
*ZMQ Socket Initializer
****************************************************************************/
void *zmq_context ;
void *mon_publisher;
void *alarm_publisher ;
void *cmd_router;
/************************** Function Prototypes ******************************/

int dpbsc_lib_init(struct DPB_I2cSensors *);
void dpbsc_lib_close(struct DPB_I2cSensors *);
int init_tempSensor (struct I2cDevice *);
int init_voltSensor (struct I2cDevice *);
int checksum_check(struct I2cDevice *,uint8_t,int);
int init_SFP_A0(struct I2cDevice *);
int init_SFP_A2(struct I2cDevice *);
int init_I2cSensors(struct DPB_I2cSensors *);
int stop_I2cSensors(struct DPB_I2cSensors *);
int init_semaphores();
int init_shared_memory();
int read_shm(int *, char *, char *);
int xlnx_ams_read_temp(int *, int, float *);
int xlnx_ams_read_volt(int *, int, float *);
int xlnx_ams_set_limits(int, const char *, const char *, float);
int mcp9844_read_temperature(struct DPB_I2cSensors *,float *);
int mcp9844_set_limits(struct DPB_I2cSensors *,int, float);
int mcp9844_set_config(struct DPB_I2cSensors *,uint8_t *,uint8_t *);
int mcp9844_interruptions(struct DPB_I2cSensors *, uint8_t );
int mcp9844_read_alarms(struct DPB_I2cSensors *);
int init_I2C_SFP(int, struct DPB_I2cSensors *);
int sfp_avago_read_temperature(struct DPB_I2cSensors *,int , float *);
int sfp_avago_read_voltage(struct DPB_I2cSensors *,int , float *);
int sfp_avago_read_lbias_current(struct DPB_I2cSensors *,int, float *);
int sfp_avago_read_tx_av_optical_pwr(struct DPB_I2cSensors *,int, float *);
int sfp_avago_read_rx_av_optical_pwr(struct DPB_I2cSensors *,int, float *);
int sfp_avago_read_status(struct DPB_I2cSensors *,int ,uint8_t *);
int sfp_avago_status_interruptions(uint8_t, int);
int sfp_avago_alarms_interruptions(struct DPB_I2cSensors *,uint16_t , int );
int sfp_avago_read_alarms(struct DPB_I2cSensors *,int ) ;
int ina3221_get_voltage(struct DPB_I2cSensors *,int , float *);
int ina3221_get_current(struct DPB_I2cSensors *,int , float *);
int ina3221_critical_interruptions(struct DPB_I2cSensors *,uint16_t , int );
int ina3221_warning_interruptions(struct DPB_I2cSensors *,uint16_t , int );
int ina3221_read_alarms(struct DPB_I2cSensors *,int);
int ina3221_set_limits(struct DPB_I2cSensors *,int ,int ,int  ,float );
int ina3221_set_config(struct DPB_I2cSensors *,uint8_t *,uint8_t *, int );
int parsing_mon_channel_data_into_object(json_object *, int, const char *, float);
int parsing_mon_channel_status_into_object(json_object *, int, const char *, int);
int parsing_mon_environment_data_into_object(json_object *, const char *, float);
int parsing_mon_environment_status_into_object(json_object *, const char *, int);
int parsing_mon_environment_string_into_object(json_object *,const char *, char *);
int alarm_json (const char*, const char *, const char *, int , float ,uint64_t ,const char *);
int status_alarm_json (const char *,const char *, int ,uint64_t ,const char *);
int command_response_json (int ,float,char *);
int command_status_response_json (int ,int,char *);
int json_schema_validate (const char *,const char *, const char *);
int get_GPIO_base_address(int *);
int write_GPIO(int , int );
int read_GPIO(int ,int *);
void unexport_GPIO();
int eth_link_status (const char *,int *);
int eth_link_status_config (char *, int );
int eth_down_alarm(const char *,int *);
int aurora_down_alarm(int ,int *);
int zmq_socket_init ();
int zmq_socket_destroy();
int dpb_command_handling(struct DPB_I2cSensors *, char **, int,char *);
int dig_command_handling(int, char *, char *);
int dig_command_translation(char *, char **, int);
int dig_command_response(char *, char *, int, char **);
int hv_lv_command_handling(char *, char *, char *);
int hv_lv_command_translation(char *, char **, int);
int hv_lv_command_response(char *, char *,int, char **);
int command_response_string_json(int, char *, char*);
int setup_serial_port(int);
int hv_read_alarms(void);
int populate_hv_hash_table(int, const char **, const char **);
int populate_lv_hash_table(int, const char **, const char **);
int populate_dig_hash_table(int, const char **);
int get_hv_hash_table_command(char *, char *);
int get_lv_hash_table_command(char *, char *);
int get_dig_hash_table_command(char **, int *);
int inList(int, int*, int);
void atexit_function();
int gen_uuid(char *);

/******************************************************************************/
/************************** Constant Definitions *****************************/
#define INA3221_NUM_CHAN 3
#define AMS_TEMP_NUM_CHAN 3
#define AMS_VOLT_NUM_CHAN 21
#define SERIAL_PORT_TIMEOUT 20 //20 deciseconds
#define SERIAL_PORT_RETRIES 5
#define SFP_NUM 6
#define DIGITIZER_0 0
#define DIGITIZER_1 1
/************************** Custom Errors Definitions *****************************/
/** @defgroup err Custom Error Flags
 *  Shared Memory content
 *  @{
 */
/** @brief Error command not valid */
#define EINCMD 1
/** @brief Error SET commnad not successful */
#define ERRSET 2
/** @brief Error READ command not successful */
#define ERRREAD 3
/** @} */
/************************** Global Flags Definitions *****************************/
int eth0_flag = 1;
int eth1_flag = 1;
int dig0_main_flag = 1;
int dig1_main_flag = 1;
int dig0_backup_flag = 1;
int dig1_backup_flag = 1;
// TODO Periodic Scan of HV,LV and digitizers to see if they are available  
int lv_connected = 0;
int hv_connected = 0;
int dig0_connected = 0;
int dig1_connected = 0;
int count_fails_until_success = 0;
int count_since_reset = 0;

/************************** SFP Related Variables *****************************/
/** @defgroup SFP_I2C SFP I2C File locations
 *  SFP I2C File locations Definitions
 *  @{
 */
/** @brief Alarm mask */
uint16_t alarms_mask[6] = {0,0,0,0,0,0};
/** @brief Status mask */
uint8_t status_mask[6] = {0,0,0,0,0,0};
/** @brief SFP I2C String array */
const char *sfp_i2c_locations[6] = {"/dev/i2c-6","/dev/i2c-10","/dev/i2c-8","/dev/i2c-12","/dev/i2c-9","/dev/i2c-13"};
/** @brief SFP connected array */
int sfp_connected[SFP_NUM] = {0,0,0,0,0,0};
/** @} */
/************************** GPIO Pins Definitions *****************************/
/** @defgroup GPIO GPIO pins
 *  GPIO pins definition
 *  @{
 */
/** @brief Number of GPIO pins used */
#define GPIO_PINS_SIZE 22

/** @brief GPIO pins definition */
#define DIG0_MAIN_AURORA_LINK 40
#define DIG0_BACKUP_AURORA_LINK 41
#define DIG1_MAIN_AURORA_LINK 42
#define DIG1_BACKUP_AURORA_LINK 43
#define SFP0_PWR_ENA 0
#define SFP1_PWR_ENA 1
#define SFP2_PWR_ENA 2
#define SFP3_PWR_ENA 3
#define SFP4_PWR_ENA 4
#define SFP5_PWR_ENA 5
#define SFP0_TX_DIS 6
#define SFP1_TX_DIS 7
#define SFP2_TX_DIS 8
#define SFP3_TX_DIS 9
#define SFP4_TX_DIS 10
#define SFP5_TX_DIS 11
#define SFP0_RX_LOS 14
#define SFP1_RX_LOS 18
#define SFP2_RX_LOS 22
#define SFP3_RX_LOS 26
#define SFP4_RX_LOS 30
#define SFP5_RX_LOS 34

/** @brief GPIO pins list */
const int GPIO_PINS[GPIO_PINS_SIZE] = {
    DIG0_MAIN_AURORA_LINK,
    DIG0_BACKUP_AURORA_LINK,
    DIG1_MAIN_AURORA_LINK,
    DIG1_BACKUP_AURORA_LINK,
    SFP0_PWR_ENA,
    SFP1_PWR_ENA,
    SFP2_PWR_ENA,
    SFP3_PWR_ENA,
    SFP4_PWR_ENA,
    SFP5_PWR_ENA,
    SFP0_TX_DIS,
    SFP1_TX_DIS,
    SFP2_TX_DIS,
    SFP3_TX_DIS,
    SFP4_TX_DIS,
    SFP5_TX_DIS,
    SFP0_RX_LOS,
    SFP1_RX_LOS,
    SFP2_RX_LOS,
    SFP3_RX_LOS,
    SFP4_RX_LOS,
    SFP5_RX_LOS
};
/** @} */

/******************************************************************************
* HV and LV GPIOs for CPUs enables
****************************************************************************/
#define LV_MAIN_CPU_GPIO_OFFSET     52
#define LV_BACKUP_CPU_GPIO_OFFSET   53
#define HV_MAIN_CPU_GPIO_OFFSET     54
#define HV_BACKUP_CPU_GPIO_OFFSET   55
#define HVLV_DRV_ENABLE_GPIO_OFFSET 56

/******************************************************************************
* Temperature Sensor Register Set - Temperature value, alarm value and alarm flags.
****************************************************************************/
#define MCP9844_TEMP_UPPER_LIM_REG 0x2
#define MCP9844_TEMP_LOWER_LIM_REG 0x3
#define MCP9844_TEMP_CRIT_LIM_REG 0x4
#define MCP9844_TEMP_UPPER_LIM 0x0
#define MCP9844_TEMP_LOWER_LIM 0x1
#define MCP9844_TEMP_CRIT_LIM 0x2
#define MCP9844_TEMP_REG 0x5
/******************************************************************************
* Temperature Sensor Register Set - Configuration and resolution value.
****************************************************************************/
#define MCP9844_RES_REG 0x9
#define MCP9844_CONFIG_REG 0x1
/******************************************************************************
* Temperature Sensor Register Set - Manufacturer and device ID.
****************************************************************************/
#define MCP9844_MANUF_ID_REG 0x6
#define MCP9844_DEVICE_ID_REG 0x7
/******************************************************************************
* SFP Register Set - Real Time Magnitudes.
****************************************************************************/
#define SFP_MSB_TEMP_REG 0x60
#define SFP_LSB_TEMP_REG 0x61
#define SFP_MSB_VCC_REG 0x62
#define SFP_LSB_VCC_REG 0x63
#define SFP_MSB_TXBIAS_REG 0x64
#define SFP_LSB_TXBIAS_REG 0x65
#define SFP_MSB_TXPWR_REG 0x66
#define SFP_LSB_TXPWR_REG 0x67
#define SFP_MSB_RXPWR_REG 0x68
#define SFP_LSB_RXPWR_REG 0x69
/******************************************************************************
* SFP Register Set - Alarms.
****************************************************************************/
#define SFP_MSB_HTEMP_ALARM_REG 0x0
#define SFP_LSB_HTEMP_ALARM_REG 0x1
#define SFP_MSB_LTEMP_ALARM_REG 0x2
#define SFP_LSB_LTEMP_ALARM_REG 0x3
#define SFP_MSB_HVCC_ALARM_REG 0x8
#define SFP_LSB_HVCC_ALARM_REG 0x9
#define SFP_MSB_LVCC_ALARM_REG 0xA
#define SFP_LSB_LVCC_ALARM_REG 0xB
#define SFP_MSB_HTXBIAS_ALARM_REG 0x10
#define SFP_LSB_HTXBIAS_ALARM_REG 0x11
#define SFP_MSB_LTXBIAS_ALARM_REG 0x12
#define SFP_LSB_LTXBIAS_ALARM_REG 0x13
#define SFP_MSB_HTXPWR_ALARM_REG 0x18
#define SFP_LSB_HTXPWR_ALARM_REG 0x19
#define SFP_MSB_LTXPWR_ALARM_REG 0x1A
#define SFP_LSB_LTXPWR_ALARM_REG 0x1B
#define SFP_MSB_HRXPWR_ALARM_REG 0x20
#define SFP_LSB_HRXPWR_ALARM_REG 0x21
#define SFP_MSB_LRXPWR_ALARM_REG 0x22
#define SFP_LSB_LRXPWR_ALARM_REG 0x23
/******************************************************************************
* SFP Register Set - Warnings.
****************************************************************************/
#define SFP_MSB_HTEMP_WARN_REG 0x4
#define SFP_LSB_HTEMP_WARN_REG 0x5
#define SFP_MSB_LTEMP_WARN_REG 0x6
#define SFP_LSB_LTEMP_WARN_REG 0x7
#define SFP_MSB_HVCC_WARN_REG 0xC
#define SFP_LSB_HVCC_WARN_REG 0xD
#define SFP_MSB_LVCC_WARN_REG 0xE
#define SFP_LSB_LVCC_WARN_REG 0xF
#define SFP_MSB_HTXBIAS_WARN_REG 0x14
#define SFP_LSB_HTXBIAS_WARN_REG 0x15
#define SFP_MSB_LTXBIAS_WARN_REG 0x16
#define SFP_LSB_LTXBIAS_WARN_REG 0x17
#define SFP_MSB_HTXPWR_WARN_REG 0xC
#define SFP_LSB_HTXPWR_WARN_REG 0xD
#define SFP_MSB_LTXPWR_WARN_REG 0xE
#define SFP_LSB_LTXPWR_WARN_REG 0xF
#define SFP_MSB_HRXPWR_WARN_REG 0x24
#define SFP_LSB_HRXPWR_WARN_REG 0x25
#define SFP_MSB_LRXPWR_WARN_REG 0x26
#define SFP_LSB_LRXPWR_WARN_REG 0x27
/******************************************************************************
* SFP Register Set - Status and flags.
****************************************************************************/
#define SFP_PHYS_DEV 0x0
#define SFP_FUNCT 0x1
#define SFP_CHECKSUM2_A0 0x40
#define SFP_STAT_REG 0x6E
#define SFP_FLG1_REG 0x70
#define SFP_FLG2_REG 0x71
#define SFP_FLG3_REG 0x74
#define SFP_FLG4_REG 0x75
/******************************************************************************
* Voltage and Current Sensor Register Set - Shunt and bus voltages
****************************************************************************/
#define INA3221_SHUNT_VOLTAGE_1_REG 0x1
#define INA3221_BUS_VOLTAGE_1_REG 0x2
#define INA3221_SHUNT_VOLTAGE_2_REG 0x3
#define INA3221_BUS_VOLTAGE_2_REG 0x4
#define INA3221_SHUNT_VOLTAGE_3_REG 0x5
#define INA3221_BUS_VOLTAGE_3_REG 0x6
/******************************************************************************
* Voltage and Current Sensor Register Set - Warnings and critical alerts
****************************************************************************/
#define INA3221_SHUNT_VOLTAGE_CRIT1_REG 0x7
#define INA3221_SHUNT_VOLTAGE_WARN1_REG 0x8
#define INA3221_SHUNT_VOLTAGE_CRIT2_REG 0x9
#define INA3221_SHUNT_VOLTAGE_WARN2_REG 0xA
#define INA3221_SHUNT_VOLTAGE_CRIT3_REG 0xB
#define INA3221_SHUNT_VOLTAGE_WARN3_REG 0xC
#define INA3221_CH1 0x0
#define INA3221_CH2 0x1
#define INA3221_CH3 0x2
/******************************************************************************
* Voltage and Current Sensor Register Set - Device Configuration, alert status configuration and enabling
****************************************************************************/
#define INA3221_CONFIG_REG 0x0
#define INA3221_MASK_ENA_REG 0xF
/******************************************************************************
* Voltage and Current Sensor Register Set - Manufacturer and device ID.
****************************************************************************/
#define INA3221_MANUF_ID_REG 0xFE
#define INA3221_DIE_ID_REG 0xFF
/******************************************************************************
*SFP set.
****************************************************************************/
#define DEV_SFP0 0x0
#define DEV_SFP1 0x1
#define DEV_SFP2 0x2
#define DEV_SFP3 0x3
#define DEV_SFP4 0x4
#define DEV_SFP5 0x5
/******************************************************************************
*INA3221 set.
****************************************************************************/
#define DEV_SFP0_2_VOLT 0x0
#define DEV_SFP3_5_VOLT 0x1
#define DEV_SOM_VOLT 0x2
/******************************************************************************
*GPIO base address
****************************************************************************/
int GPIO_BASE_ADDRESS = 0;
/******************************************************************************
*Shared Memory.
****************************************************************************/
/** @defgroup shm Shared Memory
 *  Shared Memory content
 *  @{
 */
/** @brief Shared Memory key */
#define MEMORY_KEY 7890

/** @brief Shared Memory content */
struct wrapper
{
	/** @brief Catched event type */
    char ev_type[8];
	/** @brief Catched channel type */
    char ch_type[16];
	/** @brief Catched channel */
    int chn;
	/** @brief Catched event timestamp */
    __s64 tmpstmp;
	/** @brief Semaphore that indicates there is no event catched */
    sem_t empty;
	/** @brief Semaphore that indicates event has been catched */
    sem_t full;
	/** @brief Semaphore to synchronize IIO Event monitor with main application */
    sem_t ams_sync;
};
/** @brief Shared Memory ID */
int memoryID;
/** @brief Shared Memory segment */
struct wrapper *memory;
/** @} */

#define BIT(nr) (1UL << (nr))
/******************************************************************************
*AMS channel descriptor.
****************************************************************************/
const char *ams_channels[] = {
        "PS LPD Temperature",
        "PS FPD Temperature",
        "VCC PS LPD voltage",
        "VCC PS FPD voltage",
        "PS Aux voltage reference",
        "DDR I/O VCC voltage",
        "PS IO Bank 503 voltage",
        "PS IO Bank 500 voltage",
        "VCCO_PSIO1 voltage",
        "VCCO_PSIO2 voltage",
        "VCC_PS_GTR voltage",
        "VTT_PS_GTR voltage",
        "VCC_PSADC voltage",
        "PL Temperature",
        "PL Internal voltage",
        "PL Auxiliary voltage",
        "ADC Reference P+ voltage",
        "ADC Reference N- voltage",
        "PL Block RAM voltage",
        "LPD Internal voltage",
        "FPD Internal voltage",
        "PS Auxiliary voltage",
        "PL VCCADC voltage"
    };
/******************************************************************************
LV Command Data.
****************************************************************************/
#define LV_CMD_TABLE_SIZE 10

const char *lv_daq_words[] = {
	"BDSNUM",
    "TEMP",
    "BCMTEMP",
    "RELHUM",
    "PRESS",
    "WLEAK",
    "STATUS",
    "VOLT",
    "CURR",
    "CPU",
    NULL
};

const char *lv_board_words[] = {
	"BDSNUM",
    "BDTEMP",
    "BCTEMP",
    "RH",
    "PRESSURE",
    "H2OALARM",
    "SDEN",
    "VMON",
    "IMON",
    "CPU"
};

const char *lv_mag_names[] = {
	"bdsnum",
    "boardtemp",
    "bcmtemp",
    "relathumidity",
    "pressure",
    "waterleak",
    "status",
    "voltage",
    "current",
    "cpustatus"
};

// Detected LV Serial Number
char LV_SN[8];

/******************************************************************************
HV Command Data.
****************************************************************************/

#define HV_CMD_TABLE_SIZE 12

const char *hv_daq_words[] = {
	"BDSNUM",
	"TEMP",
    "STATUS",
    "VOLT",
    "CURR",
    "CHANTEMP",
    "RAMPUP",
    "RAMPDOWN",
    "TRIP",
    "CHANERR",
    "SDVOLT",
    "SDCURR",
    NULL
};

const char *hv_board_words[] = {
	"BDSNUM",
	"BDTEMP",
    "STATUS",
    "VMON",
    "IMON",
    "TEMP",
    "RUP",
    "RDWN",
    "TRIP",
    "STATUS",
    "SDVMON",
    "SDIMON"
};

const char *hv_mag_names[] = {
	"bdsnum",
	"boardtemp",
    "status",
    "voltage", 
    "current" , 
    "temperature",
    "rampup" , 
    "rampdown" ,
    "trip", 
    "chanerr",
    "sdvolt",
    "sdcurr"
};

// Step down channels
int hv_sd_channels[] = {0,1,6,7,12,13,18,19};

// Detected HV Serial Number
char HV_SN[8];

/******************************************************************************
Digitizer Command Data.
****************************************************************************/

#define DIG_STANDARD_CMD_TABLE_SIZE 54

const char *dig_dpb_words[] = {
	"READ DISCTRES",
	"SET DISCTRES",
    "SET DISCTRES ALL",
    "READ INTTIME",
    "SET INTTIME",
    "SET INTTIME ALL",
    "READ DEADTIME",
    "SET DEADTIME",
    "SET DEADTIME ALL",
    "SET CALIB ON",
    "SET CALIB OFF",
    "READ CHSTATUS", 
    "READ CHCONTROL",
    "SET CHSTATUS", // TODO: Ask Fabrizio
    "SET FESTATUS ON",
    "SET FESTATUS OFF",
    "SET DAQSTATUS ON",
    "SET DAQSTATUS OFF",
    "SET FESTATUS ALL ON",
    "SET FESTATUS ALL OFF",
    "SET DAQSTATUS ALL ON",
    "SET DAQSTATUS ALL OFF",
    "SET TDCRST",
    "SET PEDTYPE",
    "READ PEDTYPE",
    "READ GWVER",
    "READ SWVER",
    "READ BDSTATUS",
    "READ BDCONTROL",
    "READ UPTIME",
    "SET AURORARST",
    "SET CLOCK",
    "READ DAQCLOCK",
    "READ TLNCLOCK",
    "READ RMONT",
    "SET RMONT",
    "READ RMON",
    "SET RMONRUN",
    "READ 3V3A",
    "READ 12VA",
    "READ I12V",
    "READ 5V0A",
    "READ 5V0F",
    "READ C12V",
    "READ I5VF",
    "READ I3V3A",
    "READ I12VA",
    "READ TU40",
    "READ TU41",
    "READ TU45",
    "READ BME",
    "READ TEMP",
    "READ RELHUM",
    "READ PRESS",
    NULL
};
#define DIG_MON_BOARD_CODES_SIZE 23
const int dig_monitor_mag_board_codes[] = {
    // Board monitoring
    HKDIG_GET_GW_VER,
    HKDIG_GET_SW_VER,
    HKDIG_GET_BOARD_STATUS,
    HKDIG_GET_BOARD_CNTRL,
    HKDIG_GET_UPTIME,
    HKDIG_GET_RMON_T,
    HKDIG_GET_CLOCK,
    HKDIG_GET_TLNK_LOCK,
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
	// BME280 commands
	HKDIG_GET_BME_TCAL,				// Take Temperature calibration data
	HKDIG_GET_BME_HCAL,				// Take Relative Humidity calibration data
	HKDIG_GET_BME_PCAL,				// Take Pressure calibration data
};

#define DIG_MON_CHAN_CODES_SIZE 6
const int dig_monitor_mag_chan_codes[] = {
    // Channel monitoring
    HKDIG_GET_THR_NUM,
    HKDIG_GET_IT_NUM,
    HKDIG_GET_DT_NUM,
    HKDIG_GET_CHN_STATUS,
    HKDIG_GET_CHN_CNTRL,
    HKDIG_GET_PED_TYPE,


};

const char *dig_monitor_mag_board_names[] = {
	"GWVer",
    "SWVer",
    "BDStatus",
    "BDCntrl",
    "BDUptime",
    "RMONT",
    "Clock",
    "TLock",
    "3V3A",
    "12VA",
    "I12V",
    "5V0A",
    "5V0F",
    "C12V",
    "I5VF",
    "I3V3A",
    "I12VA",
    "TU40",
    "TU41",
    "TU45",
    "boardtemp",
    "relathumidity",
    "pressure",
};

const char *dig_monitor_mag_chan_names[] = {
	"disctres",
    "inttime",
    "deadtime",
    "status",
    "cntrl",
    "pedtype"
};

// Detected Dig0 Serial Number
char DIG0_SN[32];
// Detected Dig1 Serial Number
char DIG1_SN[32];

/******************************************************************************
Hash Tables.
****************************************************************************/

struct cmd_uthash {
    char daq_word[16];                    /* key */
    char board_word[16];
    UT_hash_handle hh;         /* makes this structure hashable */
};

struct cmd_uthash *lv_cmd_table = NULL;   
struct cmd_uthash *hv_cmd_table = NULL;  

}

struct dig_uthash {
    char dpb_words[32];                    /* key */
    int dig_cmd_num;                    /* Digitizer command id */
    UT_hash_handle hh;         /* makes this structure hashable */
};

struct dig_uthash *dig_cmd_table = NULL; 
#endif
