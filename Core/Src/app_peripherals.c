/*
 * app_peripherals.c
 *
 *  Created on: Jan 6, 2026
 *      Author: marvi
 */

#include "app_shared.h"


/* Helper: BCD conversions */
static uint8_t decToBcd(uint8_t val) { return ((val / 10) << 4) | (val % 10); }
static uint8_t bcdToDec(uint8_t val) { return ((val >> 4) * 10) + (val & 0x0F); }

/* =================================================================================
 * Peripheral Callbacks
 * ================================================================================= */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (GPIO_Pin == SYSbtn_Pin){
		static uint32_t last_btn_tick = 0;
		uint32_t current_tick = HAL_GetTick();
		if ((current_tick - last_btn_tick) > BTN_DEBOUNCE_TIME) {
			osSignalSet(defaultTaskHandle, SYS_BTN_SIGNAL);
			last_btn_tick = current_tick;
			}
		}
//	appLog("[SYS] External Interrupt Triggered\r\n");
	}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	if (hadc->Instance == ADC1)
		osSignalSet(AdcConvertHandle, ADC_BUF_READY);
	}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART3)
		osSignalSet(logMessageTaskHandle, UART_TX_SIGNAL);
	}


/* =================================================================================
 * Generic I2C Implementation
 * ================================================================================= */

void I2C_Access(opcode_t op, uint16_t devAddr, uint16_t regAddr, uint16_t memAddSize, uint8_t* data, uint16_t len){
	i2cMail_t* mail = (i2cMail_t*)osMailAlloc(i2cMailHandle, osWaitForever);
	if(mail == NULL){
		appLog("[I2C] Error: Mail allocation failed\r\n");
		return;	}

	mail->op=op;
	mail->devAddr = devAddr;
	mail->regAddr = regAddr;
	mail->memAddSize = memAddSize;
	mail->pData = data;
	mail->len = len;
	mail->clientTask = osThreadGetId();

	osMailPut(i2cMailHandle, mail);
	osSignalWait(I2C_DONE_SIGNAL, osWaitForever);
	}

void StartI2cGatekeeperTask(void const* argument){
	osEvent evt;
	i2cMail_t* mail;
	appLog("[I2C] Gatekeeper Online\r\n");

	while(1){
		evt = osMailGet(i2cMailHandle, osWaitForever);
		if(evt.status == osEventMail){
			mail = (i2cMail_t*)evt.value.p;

			if(mail->op == OP_WRITE){
				if(HAL_I2C_Mem_Write(&hi2c2, mail->devAddr, mail->regAddr, mail->memAddSize, mail->pData, mail->len, 100) != HAL_OK)
					appLog("[RTC] I2C Write Fail\r\n");	}
			else if(mail->op == OP_READ){
				if (HAL_I2C_Mem_Read(&hi2c2, mail->devAddr, mail->regAddr, mail->memAddSize, mail->pData, mail->len, 100) != HAL_OK)
					appLog("[RTC] I2C Read Fail\r\n");	}
				/* Notify calling task that transfer is done */
			if(mail->clientTask != NULL)		osSignalSet(mail->clientTask, I2C_DONE_SIGNAL);
			osMailFree(i2cMailHandle, mail);	}
		}
	}

/* =================================================================================
 * Generic SPI Implementation
 * ================================================================================= */

void SPI_Access(uint8_t* txData, uint8_t* rxData, uint16_t len){
	spiMail_t* mail = (spiMail_t*)osMailAlloc(spiMailHandle, osWaitForever);
	if(mail == NULL){
		appLog("[SPI] Error: Mail allocation failed\r\n");
		return;	}

	mail->pTxData = txData;
	mail->pRxData = rxData;
	mail->len = len;
	mail->clientTask = osThreadGetId();

	osMailPut(spiMailHandle, mail);
	osSignalWait(SPI_DONE_SIGNAL, osWaitForever);
	}

void StartSpiGatekeeperTask(void const* argument){
	osEvent evt;
	spiMail_t* mail;
	appLog("[SPI] Gatekeeper Online\r\n");

	while(1){
		evt = osMailGet(spiMailHandle, osWaitForever);
		if(evt.status == osEventMail){
			mail = (spiMail_t*)evt.value.p;
			/* NOTE : Manual Chip Select Assert not needed: Hardware NSS is enabled */
				/* Full-Duplex transaction */
			if(mail->pTxData != NULL && mail->pRxData != NULL)		HAL_SPI_TransmitReceive(&hspi3, mail->pTxData, mail->pRxData, mail->len, 100);
				/* Tx only */
			else if(mail->pTxData != NULL)		HAL_SPI_Transmit(&hspi3, mail->pTxData, mail->len, 100);
				/* Rx only */
			else if(mail->pRxData != NULL)		HAL_SPI_Receive(&hspi3, mail->pRxData, mail->len, 100);
				/* Signal completion */
			if(mail->clientTask != NULL)		osSignalSet(mail->clientTask, SPI_DONE_SIGNAL);
			osMailFree(spiMailHandle, mail);	}
		}
	}

/* =================================================================================
 * RTC specific implementation
 * ================================================================================= */

void RTC_WriteTime(rtc_time_t* t){
	uint8_t buf[7];
	buf[0] = decToBcd(t->sec);
	buf[1] = decToBcd(t->min);
	buf[2] = decToBcd(t->hour);
	buf[3] = decToBcd(t->wday);
	buf[4] = decToBcd(t->day);
	buf[5] = decToBcd(t->month);
	buf[6] = decToBcd(t->year);

	I2C_Access(OP_WRITE, RTC_I2C_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, buf, 7);
	appLog("[RTC] Time updated to 20%02d-%02d-%02d %02d:%02d:%02d\r\n", t->year, t->month, t->day, t->hour, t->min, t->sec);
	}

void RTC_GetTime(rtc_time_t* t){
	uint8_t buf[7];
	xEventGroupWaitBits(sysEventGroup, SYS_BIT_TIME_SYNC, pdFALSE, pdTRUE, osWaitForever);
	I2C_Access(OP_READ, RTC_I2C_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, buf, 7);

	t->sec   = bcdToDec(buf[0] & 0x7F);
	t->min   = bcdToDec(buf[1]);
	t->hour  = bcdToDec(buf[2]);
	t->wday  = bcdToDec(buf[3]);
	t->day   = bcdToDec(buf[4]);
	t->month = bcdToDec(buf[5]);
	t->year  = bcdToDec(buf[6]);
	}

/* =================================================================================
 * FRAM specific implementation
 * ================================================================================= */

void FRAM_Write(uint16_t addr, uint8_t* pData, uint16_t len){
	uint8_t opCode = FRAM_WREN;
	static uint8_t txBuf[SPI_BUF_SIZE];
	if(len > SPI_BUF_SIZE-3)		len = SPI_BUF_SIZE-3;			// Limit len to fit buffer (32 - 3 cmd bytes)
	SPI_Access(&opCode, NULL, 1);			// Send write enable
	 /* Send write op + addr + data */
	txBuf[0] = FRAM_WRITE;
	txBuf[1] = (uint8_t)((addr >> 8) & 0xFF);
	txBuf[2] = (uint8_t)(addr & 0xFF);
	memcpy(&txBuf[3], pData, len);

	SPI_Access(txBuf, NULL, len+3);
	// appLog("[RAM] Write %d bytes @ 0x%04X\r\n", len, addr);
	}

void FRAM_Read(uint16_t addr, uint8_t* pData, uint16_t len){
	static uint8_t txBuf[SPI_BUF_SIZE] = {0};
	static uint8_t rxBuf[SPI_BUF_SIZE] = {0};
	if(len > SPI_BUF_SIZE-3)		len = SPI_BUF_SIZE-3;			// Limit len to fit buffer (32 - 3 cmd bytes)
		/* Prepare opCode */
	txBuf[0] = FRAM_READ;
	txBuf[1] = (uint8_t)((addr >> 8) & 0XFF);
	txBuf[2] = (uint8_t)(addr & 0xFF);

	SPI_Access(txBuf, rxBuf, len+3);
	memcpy(pData, &rxBuf[3], len);	// Extract data (skip first 3 bytes of garbage return during addr tx)
	}

/* =================================================================================
 * Misc peripheral tasks
 * ================================================================================= */

void StartHeartBeatTask(void const * argument){
	uint32_t previousWakeTime;
	while(1){
		waitForRunSignal();
		previousWakeTime = osKernelSysTick();
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		osDelayUntil(&previousWakeTime, HEARTBEAT_INTERVAL);	}
	}
