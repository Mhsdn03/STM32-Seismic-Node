/*
 * app_logger.c
 *
 *  Created on: Jan 6, 2026
 *      Author: marvi
 */

#include "app_shared.h"


void appLog(const char* fmt, ...){
	  /* Request a block of memory from pre-allocated pool, no timeout */
	logMail_t* mail = (logMail_t*)osMailAlloc(logMailHandle, 0);
	if (mail == NULL)		return;			// Pool full, dropping log

	va_list args;
	va_start(args, fmt);
	vsnprintf(mail->buffer, LOG_MAX_STR_LEN, fmt, args);
	va_end(args);
	osMailPut(logMailHandle, mail);
	}

/**
* @brief Function implementing the logMessageTask thread.
* @param argument: Not used
* @retval None
*/
void LogMessageTask(void const * argument){
	osEvent evt;
	appLog("[SYS] MailQ Logging Online\r\n");

	while(1){
		if ((evt = osMailGet(logMailHandle, osWaitForever)).status == osEventMail){
			logMail_t* mail = (logMail_t*)evt.value.p;
			if (mail != NULL){
				if(HAL_UART_Transmit_DMA(&huart3, (uint8_t*)mail->buffer, strlen(mail->buffer)) == HAL_OK)
					osSignalWait(UART_TX_SIGNAL, osWaitForever);
				osMailFree(logMailHandle, mail);	}
			}
		}
	}

void StartStackMonitorTask(void const* argument){
	appLog("[SYS] Stack Monitor Task Started\r\n");
	while(1){
		waitForRunSignal();
			/* Log High Water Marks */
		appLog("[STK] Free words:\r\n\tDef:%u\r\n\tLog:%u\r\n\tAdc:%u\r\n\tUDP:%u\r\n\tSTK:%u\r\n\tCLI:%u\r\n\tSRV:%u\r\n\tNET:%u\r\n\tLST:%u\r\n\tJSN:%u\r\n\tLED:%u\r\n",
					(unsigned int)uxTaskGetStackHighWaterMark(defaultTaskHandle),
					(unsigned int)uxTaskGetStackHighWaterMark(logMessageTaskHandle),
					(unsigned int)uxTaskGetStackHighWaterMark(AdcConvertHandle),
					(unsigned int)uxTaskGetStackHighWaterMark(UDPHeartbeatHandle),
					(unsigned int)uxTaskGetStackHighWaterMark(stackMonitorTaskHandle),
					(unsigned int)uxTaskGetStackHighWaterMark(clientTaskHandle),
					(unsigned int)uxTaskGetStackHighWaterMark(serverTaskHandle),
					(unsigned int)uxTaskGetStackHighWaterMark(linkMonitorTaskHandle),
					(unsigned int)uxTaskGetStackHighWaterMark(broadcastListenerTaskHandle),
					(unsigned int)uxTaskGetStackHighWaterMark(JsonProcessingTaskHandle),
					(unsigned int)uxTaskGetStackHighWaterMark(heartBeatTaskHandle));
		osDelay(120000);	}
	}
