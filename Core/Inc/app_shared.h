/*
 * app_shared.h
 *
 *  Created on: Jan 6, 2026
 *      Author: marvi
 */

#ifndef INC_APP_SHARED_H_
#define INC_APP_SHARED_H_

/* =================================================================================
 * Headers
 * ================================================================================= */
/* STM32 Auto generated headers */
#include "main.h"
#include "cmsis_os.h"
#include "lwip.h"
/* Additional headers */
#include "string.h"
#include "stdio.h"
#include "stdarg.h"
#include "math.h"
#include "lwip/sockets.h"
#include "lwip/netif.h"
#include "lwip/netdb.h"
#include "event_groups.h"
#include "time.h"
#include "dns.h"

/* =================================================================================
 * Defines
 * ================================================================================= */
	/* String lengths */
#define MSG_MAX_STR_LEN 	256UL
#define LOG_MAX_STR_LEN		256UL
	/* Signal values */
#define ADC_BUF_READY				0x01
#define UART_TX_SIGNAL			0x01
#define SYS_BIT_RUNNING			(1<<0)
#define SYS_BIT_LINK_UP			(1<<1)
#define SYS_BIT_TIME_SYNC		(1<<2)
#define I2C_DONE_SIGNAL   	(1<<14)
#define SPI_DONE_SIGNAL			(1<<15)
	/* User button debounce */
#define SYS_BTN_SIGNAL			0x01
#define BTN_DEBOUNCE_TIME  	200UL
	/* ADC processing */
#define ADC_CHANNELS 					3UL
#define SAMPLES_PER_CHANNEL 	10UL
#define ADC_SAMPLES						(ADC_CHANNELS * SAMPLES_PER_CHANNEL)
#define MAX_PEAKS							10UL
	/* Mail sizeq */
#define LOG_MAIL_SIZE		16UL
#define MSG_MAIL_SIZE		16UL
#define I2C_MAIL_SIZE 	8UL
#define SPI_MAIL_SIZE		8UL
	/* Timings & intervals */
#define HEARTBEAT_INTERVAL 	1000UL
#define UDP_HEARTBEAT_INTERVAL 	10000UL
#define RETRY_INTERVAL    			2000UL
#define LINK_CHECK_INTERVAL			1000UL
	/* Network defines */
#define BROADCAST_PORT  			12345UL
#define TCP_PORT 							12345UL
#define CONN_TIMEOUT_US 			200000UL
#define CONN_TIMEOUT_MS				200UL
#define PEER_TIMEOUT_MS				15000UL
#define MAX_TARGET_IPS				8UL
	/* NTP defines */
#define NTP_SERVER_NAME       "be.pool.ntp.org"
#define NTP_PORT              123
#define NTP_PACKET_SIZE       48
#define NTP_TIMESTAMP_DELTA   2208988800UL
#define TIMEZONE_OFFSET_HRS   1
	/* i2c addresses */
#define RTC_I2C_ADDR          (0xD0)
	/* FRAM defines */
#define FRAM_WREN  							0x06
#define FRAM_WRITE 							0x02
#define FRAM_READ  							0x03
#define FRAM_RDID  							0x9F
#define FRAM_PEAK_ADDR					0x0010
#define FRAM_REMOTE_DATA_ADDR 	0x0200
#define FRAM_INIT_MAGIC_ADDR    0x01FC
#define FRAM_MAGIC_VALUE        0x42
	/* Buffer sizes */
#define NET_BUF_SIZE		256UL
#define SPI_BUF_SIZE		256UL
	/* Devide info */
#define DEVICE_ID			"nucleo-12"
#define DEVICE_IP			"192.168.1.191"
	/* xEventGroup macros */
#define waitForRunSignal()	\
		xEventGroupWaitBits(sysEventGroup, SYS_BIT_RUNNING, pdFALSE, pdTRUE, portMAX_DELAY)
#define waitForNetworkAvailable() \
		xEventGroupWaitBits(sysEventGroup, SYS_BIT_RUNNING | SYS_BIT_LINK_UP, pdFALSE, pdTRUE, portMAX_DELAY)

/* typedef ------------------------------------------------------------*/
typedef enum{
	OP_READ,
	OP_WRITE
	}	opcode_t;

typedef enum{
  CLI_STATE_DISCONNECTED,
	CLI_STATE_CONNECTING,
	CLI_STATE_COMMUNICATE,
	CLI_STATE_ERROR_RECOVERY
	}	cliState_t;

typedef enum{
	SRV_STATE_OFFLINE,
	SRV_STATE_LISTENING,
	SRV_STATE_COMMUNICATE,
	SRV_STATE_ERROR_RECOVERY
	} srvState_t;

typedef enum{
	LST_STATE_OFFLINE,
	LST_STATE_LISTENING,
	LST_STATE_PROCESSING,
	LST_STATE_ERROR_RECOVERY
	}	lstState_t;

typedef enum{
	UDP_STATE_OFFLINE,
	UDP_STATE_BROADCASTING,
	UDP_STATE_ERROR_RECOVERY
	}	udpState_t;

typedef enum{
	MSG_TYPE_UNKNOWN = 0,
	MSG_TYPE_DATA,
	MSG_TYPE_PRESENCE,
	MSG_TYPE_ALERT
	} msgType_t;

typedef struct{
	char ip[16];
	char id[10];
	uint32_t last_seen;
	}	peer_t;

typedef struct {
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
	uint8_t day;
	uint8_t wday;
	uint8_t month;
	uint8_t year;
	} rtc_time_t;

typedef struct {
	rtc_time_t t;
	float x, y, z;
	float mag;
	} peak_record_t;

typedef struct {
	char id[10];
	peak_record_t data;
} remote_data_entry_t;

typedef struct{
	char buffer[LOG_MAX_STR_LEN];
	} logMail_t;

typedef struct{
	char buffer[LOG_MAX_STR_LEN];
	}	msgMail_t;

typedef struct{
	opcode_t op;
	uint16_t devAddr;
	uint16_t regAddr;
	uint16_t memAddSize;
	uint8_t* pData;
	uint16_t len;
	osThreadId clientTask;
	}	i2cMail_t;

typedef struct{
	uint8_t* pTxData;
	uint8_t* pRxData;
	uint16_t len;
	osThreadId clientTask;
	}	spiMail_t;

/* Variables (RTOS Handles) -----------------------------------------*/
extern osThreadId defaultTaskHandle;
extern osThreadId logMessageTaskHandle;
extern osThreadId clientTaskHandle;
extern osThreadId serverTaskHandle;
extern osThreadId heartBeatTaskHandle;
extern osThreadId AdcConvertHandle;
extern osThreadId UDPHeartbeatHandle;
extern osThreadId linkMonitorTaskHandle;
extern osThreadId stackMonitorTaskHandle;
extern osThreadId broadcastListenerTaskHandle;
extern osThreadId JsonProcessingTaskHandle;
extern osThreadId NtpSyncTaskHandle;
extern osThreadId i2cGatekeeperHandle;
extern osThreadId spiGatekeeperHandle;

extern osMutexId ipConfigMutexHandle;

extern EventGroupHandle_t sysEventGroup;

extern osMailQId logMailHandle;
extern osMailQId msgMailHandle;
extern osMailQId i2cMailHandle;
extern osMailQId spiMailHandle;

extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim6;
extern I2C_HandleTypeDef hi2c2;
extern SPI_HandleTypeDef hspi3;

/* Global data -----------------------------------------*/
	/* ADC data */
extern uint16_t adcBuffer[ADC_SAMPLES];
extern peak_record_t high_scores[MAX_PEAKS];
	/* Network peer data */
extern peer_t peerList[MAX_TARGET_IPS];
extern uint8_t ip_count;

/* Shared Function Prototypes */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

void appLog(const char* fmt, ...);
void msgProcess(const char* data);
void getJSONval(const char* json, const char* key, char* out_val, size_t max_len);
void JSONsubstring(const char* json, const char* key, char* out_val, size_t max_len);
void RTC_WriteTime(rtc_time_t* t);
void RTC_GetTime(rtc_time_t* t);
void FRAM_Write(uint16_t addr, uint8_t* pData, uint16_t len);
void FRAM_Read(uint16_t addr, uint8_t* pData, uint16_t len);
void I2C_Access(opcode_t op, uint16_t devAddr, uint16_t regAddr, uint16_t memAddSize, uint8_t* data, uint16_t len);
void SPI_Access(uint8_t* txData, uint8_t* rxData, uint16_t len);

#endif /* INC_APP_SHARED_H_ */
