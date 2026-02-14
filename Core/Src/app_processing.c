/*
 * app_processing.c
 *
 *  Created on: Jan 6, 2026
 *      Author: marvi
 */

#include "app_shared.h"


	/* global variables */
peak_record_t high_scores[MAX_PEAKS];
uint16_t adcBuffer[ADC_SAMPLES];
static remote_data_entry_t remote_entries[MAX_TARGET_IPS];

void JSONsubstring(const char* json, const char* key, char* out_val, size_t max_len){
	char searchKey[32];
	snprintf(searchKey, sizeof(searchKey), "\"%s\"", key);
	char* pos = strstr(json, searchKey);			// Search key position in JSON
		/* Return if key doesn't exist */
	if(!pos){
		out_val[0] = '\0';
		return;	}
		/* move past key and find colon */
	pos += strlen(searchKey);
	if(!(pos=strchr(pos, ':'))){
		out_val[0] = '\0';
		return;	}
	pos++;
		/* Skip whitespace */
	while((*pos ==  ' ' || *pos == '\"' || *pos == '\t') && *pos != '\0')		pos++;
		/* Copy until closing symbol */
	size_t i = 0;
	while(*pos != '\0' && *pos != '\"' && *pos != ',' && *pos != '}' && i < max_len-1)		out_val[i++] = *pos++;
	out_val[i] = '\0';
	}

void msgProcess(const char* data){
	msgMail_t* mail = (msgMail_t*)osMailAlloc(msgMailHandle, 0);
	if(mail == NULL){
		appLog("[MSG] Mail queue full, dropping message\r\n");
		return;	}
	strncpy(mail->buffer, data, MSG_MAX_STR_LEN-1);
	mail->buffer[MSG_MAX_STR_LEN-1] = '\0';
	osMailPut(msgMailHandle, mail);
	}

void StartJsonProcessingTask(void const* argument){
	char tempVal_buf[32];
	char id_buf[10];
	char ip_buf[16];
	msgType_t currMsgType;
	osEvent evt;

	/* --- CHECK & INITIALIZE FRAM --- */
	uint8_t magic_check = 0;
	FRAM_Read(FRAM_INIT_MAGIC_ADDR, &magic_check, 1);

	if(magic_check != FRAM_MAGIC_VALUE){
		appLog("[RAM] Invalid Magic (0x%02X). Formatting remote storage...\r\n", magic_check);
		/* Clear local cache */
		memset(remote_entries, 0, sizeof(remote_entries));
		/* Clear FRAM storage */
		FRAM_Write(FRAM_REMOTE_DATA_ADDR, (uint8_t*)remote_entries, sizeof(remote_entries));
		/* Write valid magic flag */
		magic_check = FRAM_MAGIC_VALUE;
		FRAM_Write(FRAM_INIT_MAGIC_ADDR, &magic_check, 1);
		appLog("[RAM] Format complete.\r\n");	}
	else{
		/* Pre-load FRAM data into global RAM cache */
		FRAM_Read(FRAM_REMOTE_DATA_ADDR, (uint8_t*)remote_entries, sizeof(remote_entries));
		appLog("[MSG] Remote data cache loaded from FRAM\r\n");	}

	appLog("[MSG] Network JSON message processing online\r\n");

	while(1){
		if ((evt = osMailGet(msgMailHandle, osWaitForever)).status == osEventMail){
			msgMail_t* mail = (msgMail_t*)evt.value.p;
			if (mail != NULL){
				mail->buffer[sizeof(mail->buffer)-1] = '\0';
				currMsgType = MSG_TYPE_UNKNOWN;
					/* Message type determination */
				JSONsubstring(mail->buffer, "type", tempVal_buf, sizeof(tempVal_buf));
				if(strcmp(tempVal_buf, "data") == 0)		currMsgType = MSG_TYPE_DATA;
				else if (strcmp(tempVal_buf, "presence") == 0)		currMsgType = MSG_TYPE_PRESENCE;

					/* Message processing */
				switch(currMsgType){
				case MSG_TYPE_PRESENCE:
					JSONsubstring(mail->buffer, "id", id_buf, sizeof(id_buf));
					JSONsubstring(mail->buffer, "ip", ip_buf, sizeof(ip_buf));
						/* Validate received IP address */
					struct sockaddr_in sa_check;
					if(strlen(ip_buf) > 0 && inet_pton(AF_INET, ip_buf, &(sa_check.sin_addr)) == 1){
						osMutexWait(ipConfigMutexHandle, osWaitForever);
							/* check if peer exists/update timestamp */
						int found_idx = -1;
						for(int i = 0; i < ip_count; i++)
							if(strncmp(peerList[i].ip, ip_buf, 16) == 0){
								found_idx = i;
								break;	}
							/* update known peer timestamp */
						if(found_idx >=0)		peerList[found_idx].last_seen = HAL_GetTick();
						else if(ip_count < MAX_TARGET_IPS){
							strncpy(peerList[ip_count].ip, ip_buf, 16);
							peerList[ip_count].ip[15] = '\0';
							peerList[ip_count].last_seen = HAL_GetTick();
							strncpy(peerList[ip_count].id, id_buf, 10);
							peerList[ip_count].id[9] = '\0';
							ip_count++;
							appLog("[MSG] Found peer: %s @ %s\r\n", id_buf, ip_buf);	}
						osMutexRelease(ipConfigMutexHandle);	}
					else if (strlen(ip_buf) > 0)		appLog("[MSG] Invalid IP, ignored: %s\r\n", ip_buf);
					break;

				case MSG_TYPE_DATA:	{
					peak_record_t remoteData;
					char val_buf[16];
						/* Extract sender ID */
					JSONsubstring(mail->buffer, "id", id_buf, sizeof(id_buf));
					appLog("[MSG] Rx Data from %s\r\n", id_buf);
						/* Extract Acceleration Data */
					JSONsubstring(mail->buffer, "x", val_buf, sizeof(val_buf));
					remoteData.x = atof(val_buf);
					JSONsubstring(mail->buffer, "y", val_buf, sizeof(val_buf));
					remoteData.y = atof(val_buf);
					JSONsubstring(mail->buffer, "z", val_buf, sizeof(val_buf));
					remoteData.z = atof(val_buf);
						/* Calculate magnitude for completeness */
					remoteData.mag = sqrtf(remoteData.x*remoteData.x + remoteData.y*remoteData.y + remoteData.z*remoteData.z);
						/* Timestamp it with local time */
					RTC_GetTime(&remoteData.t);

					int idx = -1;
					int empty_idx = -1;

					for(int i = 0; i < MAX_TARGET_IPS; i++){
							/* check for match in global cache */
						if(strncmp(remote_entries[i].id, id_buf, 10) == 0){
							idx = i;
							break;	}
							/* check for first empty slot */
						if(empty_idx == -1 && remote_entries[i].id[0] == 0)		empty_idx = i;
						}
					if(idx != -1){
							/* update existing slot in FRAM */
						remote_entries[idx].data = remoteData;
						FRAM_Write(FRAM_REMOTE_DATA_ADDR + (idx * sizeof(remote_data_entry_t)),
								   (uint8_t*)&remote_entries[idx], sizeof(remote_data_entry_t));
						appLog("[RAM] Updated data for node %s (Mag: %.2f)\r\n", id_buf, remoteData.mag);	}
					else if(empty_idx != -1){
							/* Create new slot in RAM */
						strncpy(remote_entries[empty_idx].id, id_buf, 10);
						remote_entries[empty_idx].data = remoteData;
							/* Write ONLY this slot to FRAM */
						FRAM_Write(FRAM_REMOTE_DATA_ADDR + (empty_idx * sizeof(remote_data_entry_t)),
								   (uint8_t*)&remote_entries[empty_idx], sizeof(remote_data_entry_t));
						appLog("[RAM] New node data stored for %s (Mag: %.2f)\r\n", id_buf, remoteData.mag);
						}
					else		appLog("[RAM] Storage full. Ignored data from %s\r\n", id_buf);
					break;	}

				default:
					appLog("[MSG] Message type unknown: %s\r\n", tempVal_buf);
					break;
					}
				osMailFree(msgMailHandle, mail);
				}
			}
		}
	}


void StartAdcConvert(void const * argument){
	float avg_sq_x, avg_sq_y, avg_sq_z;
	uint32_t sum_sq_x, sum_sq_y, sum_sq_z;
	float total_mag;
	FRAM_Read(FRAM_PEAK_ADDR, (uint8_t*)high_scores, sizeof(high_scores));
	rtc_time_t t;

	while(1){
		if(osSignalWait(ADC_BUF_READY, osWaitForever).status == osEventSignal){
			sum_sq_x = 0;
			sum_sq_y = 0;
			sum_sq_z = 0;

			for(uint8_t i = 0; i < SAMPLES_PER_CHANNEL; i++){
				uint16_t vx = adcBuffer[i*ADC_CHANNELS];
				uint16_t vy = adcBuffer[i*ADC_CHANNELS + 1];
				uint16_t vz = adcBuffer[i*ADC_CHANNELS + 2];

				sum_sq_x += (uint32_t)vx *vx;
				sum_sq_y += (uint32_t)vy *vy;
				sum_sq_z += (uint32_t)vz *vz;
				}
			avg_sq_x = sqrtf((float)sum_sq_x / SAMPLES_PER_CHANNEL);
			avg_sq_y = sqrtf((float)sum_sq_y / SAMPLES_PER_CHANNEL);
			avg_sq_z = sqrtf((float)sum_sq_z / SAMPLES_PER_CHANNEL);

			total_mag = sqrtf(avg_sq_x*avg_sq_x + avg_sq_y*avg_sq_y + avg_sq_z*avg_sq_z);
			if(total_mag > high_scores[MAX_PEAKS-1].mag){
				RTC_GetTime(&t);
				for(uint8_t j = 0; j < MAX_PEAKS; j++){
						/* Shift lower scores down to make room */
					if(total_mag > high_scores[j].mag){
						if(j < MAX_PEAKS-1)
							memmove(&high_scores[j+1], &high_scores[j], sizeof(peak_record_t)*(MAX_PEAKS-1-j));
							/* Insert new record */
						high_scores[j].mag = total_mag;
						high_scores[j].t = t;
						high_scores[j].x = avg_sq_x;
						high_scores[j].y = avg_sq_y;
						high_scores[j].z = avg_sq_z;
							/* save updated table to FRAM */
						FRAM_Write(FRAM_PEAK_ADDR, (uint8_t*)high_scores, sizeof(high_scores));
						appLog("[ADC] New High Score: %.2f\r\n", total_mag);
						break;	}
					}
				}
			HAL_GPIO_TogglePin(GPIOB, LD1_Pin);	}
		}
	}
