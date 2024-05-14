#ifndef __DPB2SC_H_INCLUDED__
#define __DPB2SC_H_INCLUDED__


#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include "timer.h"
#include <semaphore.h>
#include <sys/shm.h>
#include <fcntl.h>
#include <time.h>
#include <zmq.h>
#include "json-c/json.h"
#include <math.h>
#include <dirent.h>
#include <signal.h>
#include <regex.h>

#include "i2c.h"
#include "linux/errno.h"
/************************** Main Struct Definition *****************************/

struct DPB_I2cSensors{

	struct I2cDevice dev_pcb_temp;
	struct I2cDevice dev_sfp0_2_volt;
	struct I2cDevice dev_sfp3_5_volt;
	struct I2cDevice dev_som_volt;
	struct I2cDevice dev_sfp0_A0;
	struct I2cDevice dev_sfp1_A0;
	struct I2cDevice dev_sfp2_A0;
	struct I2cDevice dev_sfp3_A0;
	struct I2cDevice dev_sfp4_A0;
	struct I2cDevice dev_sfp5_A0;
	struct I2cDevice dev_sfp0_A2;
	struct I2cDevice dev_sfp1_A2;
	struct I2cDevice dev_sfp2_A2;
	struct I2cDevice dev_sfp3_A2;
	struct I2cDevice dev_sfp4_A2;
	struct I2cDevice dev_sfp5_A2;
	//struct I2cDevice dev_mux0;
	//struct I2cDevice dev_mux1;
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

/** @brief Semaphore to synchronize thread creation */
sem_t thread_sync;

/** @brief Semaphore to synchronize GPIO file accesses */
sem_t file_sync;

/** @} */
/******************************************************************************
*Child process and threads.
****************************************************************************/
/** @defgroup pth Child process and threads
 *  Threads and subprocesses declaration
 *  @{
 */
/** @brief IIO Event Monitor Process ID */
pid_t child_pid;
/** @} */

/******************************************************************************
*ZMQ Socket Initializer
****************************************************************************/
void *zmq_context ;
void *mon_publisher;
void *alarm_publisher ;
void *cmd_router;
/************************** Function Prototypes ******************************/

int init_tempSensor (struct I2cDevice *);
int init_voltSensor (struct I2cDevice *);
int checksum_check(struct I2cDevice *,uint8_t,int);
int init_SFP_A0(struct I2cDevice *);
int init_SFP_A2(struct I2cDevice *);
int init_I2cSensors(struct DPB_I2cSensors *);
int stop_I2cSensors(struct DPB_I2cSensors *);
int iio_event_monitor_up();
int xlnx_ams_read_temp(int *, int, float *);
int xlnx_ams_read_volt(int *, int, float *);
int xlnx_ams_set_limits(int, char *, char *, float);
int mcp9844_read_temperature(struct DPB_I2cSensors *,float *);
int mcp9844_set_limits(struct DPB_I2cSensors *,int, float);
int mcp9844_set_config(struct DPB_I2cSensors *,uint8_t *,uint8_t *);
int mcp9844_interruptions(struct DPB_I2cSensors *, uint8_t );
int mcp9844_read_alarms(struct DPB_I2cSensors *);
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
int parsing_mon_sensor_data_into_array (json_object *,float , char *, int );
int parsing_mon_status_data_into_array(json_object *, int , char *,int );
int alarm_json (char*,char *,char *, int , float ,uint64_t ,char *);
int status_alarm_json (char *,char *, int ,uint64_t ,char *);
int command_response_json (int ,float );
int command_status_response_json (int ,int );
int json_schema_validate (char *,const char *, char *);
int get_GPIO_base_address(int *);
int write_GPIO(int , int );
int read_GPIO(int ,int *);
void unexport_GPIO();
int eth_link_status (char *,int *);
int eth_link_status_config (char *, int );
int eth_down_alarm(char *,int *);
int aurora_down_alarm(int ,int *);
int zmq_socket_init ();
int dpb_command_handling(struct DPB_I2cSensors *, char **, int);
void dig_command_handling(char **);
void hv_command_handling(char **);
void lv_command_handling(char **);
void atexit_function();
void sighandler(int );
int gen_uuid(char *);

/******************************************************************************/
/************************** Constant Definitions *****************************/
#define INA3221_NUM_CHAN 3
#define AMS_TEMP_NUM_CHAN 3
#define AMS_VOLT_NUM_CHAN 21
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
int break_flag = 0;
int sfp0_connected = 0;
int sfp1_connected = 0;
int sfp2_connected = 0;
int sfp3_connected = 0;
int sfp4_connected = 0;
int sfp5_connected = 0;
/************************** SFP Alarms Masks Definitions *****************************/
/** @defgroup SFP_Masks SFP Alarms Masks
 *  SFP Alarms Masks Definitions
 *  @{
 */
/** @brief Alarm mask */
uint16_t alarms_mask[6] = {0,0,0,0,0,0};
/** @brief Status mask */
uint8_t status_mask[6] = {0,0,0,0,0,0};
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
*Threads timers (ms).
****************************************************************************/
#define MONIT_THREAD_PERIOD 5000000
#define ALARMS_THREAD_PERIOD 100
#define AMS_ALARMS_THREAD_PERIOD 100
#define COMMAND_THREAD_PERIOD 50
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

/******************************************************************************
*AMS channel descriptor.
****************************************************************************/
char *ams_channels[] = {
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


#endif
