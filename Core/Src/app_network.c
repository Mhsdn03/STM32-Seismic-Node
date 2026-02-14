/*
 * app_network.c
 *
 *  Created on: Jan 6, 2026
 *      Author: marvi
 */

#include "app_shared.h"


	/* multi IP storage */
peer_t peerList[MAX_TARGET_IPS] = {0};
uint8_t ip_count = 0;

	/* Private prototype */
static void prune_dead_peers(void);

/* --- Tasks --- */
void StartLinkMonitorTask(void const* argument){
	appLog("[SYS] Link Monitor Task Started\r\n");
	while(1){
		waitForRunSignal();
		if(netif_is_link_up(netif_default)){
			if(!(xEventGroupGetBits(sysEventGroup) & SYS_BIT_LINK_UP)){
				xEventGroupSetBits(sysEventGroup, SYS_BIT_LINK_UP);
				appLog("[NET] LINK UP. Network Services Enabled\r\n");	}
			}
		else{
			if(xEventGroupGetBits(sysEventGroup) & SYS_BIT_LINK_UP){
				xEventGroupClearBits(sysEventGroup, SYS_BIT_LINK_UP);
				appLog("[NET] LINK DOWN. Network Services Paused\r\n");	}
			}
		prune_dead_peers();
		osDelay(LINK_CHECK_INTERVAL);
		}
	}

/**
* @brief Function implementing the clientTask thread.
* @param argument: Not used
* @retval None
*/
void StartClientTask(void const * argument){
	struct sockaddr_in serv_addr = {0};
	struct pollfd fds[1];
	int sock = -1;
	int ret;
	static char buf[NET_BUF_SIZE];			// Unified buffer for TX/RX
	char target_ip_local[16];
	int ip_idx = 0;			// local index for IP list
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(TCP_PORT);
	cliState_t state = CLI_STATE_DISCONNECTED;
	appLog("[CLI] Client Task Started\r\n");

  while(1){
    switch(state){
		case CLI_STATE_DISCONNECTED:
			waitForNetworkAvailable();
			osMutexWait(ipConfigMutexHandle, osWaitForever);
			if(ip_count == 0){
				osMutexRelease(ipConfigMutexHandle);
				appLog("[CLI] No peers found, yet...\r\n");
				ip_idx = 0;
				osDelay(UDP_HEARTBEAT_INTERVAL);
				break;	}
			if(ip_idx >= ip_count)		ip_idx = 0;
			if(ip_idx == 0){
				osMutexRelease(ipConfigMutexHandle);			// release mutex when sleeping
				appLog("[CLI] Peers found. Accumulating data (60s)...\r\n");
				osDelay(60000);
				osMutexWait(ipConfigMutexHandle, osWaitForever);
					/* re-verify ip count aster sleep */
				if(ip_count == 0){
					osMutexRelease(ipConfigMutexHandle);
					break;	}
				}
				/* get current target IP */
			strncpy(target_ip_local, peerList[ip_idx].ip, 16);
			target_ip_local[15] = '\0';
			ip_idx++;
			osMutexRelease(ipConfigMutexHandle);
			serv_addr.sin_addr.s_addr = inet_addr(target_ip_local);
				/* Create non-blocking socket */
      if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
				appLog("[CLI] Socket creation failed\r\n");
				state = CLI_STATE_ERROR_RECOVERY;
      	break;	}
      fcntl(sock, F_SETFL, fcntl(sock, F_GETFL, 0) | O_NONBLOCK);
      	/* Initiate connection */
      ret = connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr));
      if(ret == 0){
      	state = CLI_STATE_COMMUNICATE;			// Immediate connection
      	break;	}
      else if(ret < 0 && errno == EINPROGRESS)		state = CLI_STATE_CONNECTING;
      else{
      	appLog("[CLI] Connection failed, errno %d\r\n", errno);
      	state = CLI_STATE_ERROR_RECOVERY;
      	break;	}

		case CLI_STATE_CONNECTING:
			fds[0].fd = sock;
			fds[0].events = POLLOUT;			// wait for writable state
			ret = poll(fds, 1, CONN_TIMEOUT_MS);
			if(ret == 0){
				appLog("[CLI] Connection timeout, retrying...\r\n");
				state = CLI_STATE_ERROR_RECOVERY;	}
			else if(ret < 0){
				appLog("[CLI] Poll error during connect\r\n");
				state = CLI_STATE_ERROR_RECOVERY;	}
			else{
					/* check error flag */
				int so_error = 0;
				socklen_t len = sizeof(so_error);
				getsockopt(sock, SOL_SOCKET, SO_ERROR, &so_error, &len);

				if(so_error == 0 && (fds[0].revents & POLLOUT)){
					appLog("[CLI] Connected to %s\r\n", target_ip_local);
					state = CLI_STATE_COMMUNICATE;	}
				else{
          appLog("[CLI] Connect failed, so_error %d\r\n", so_error);
          state = CLI_STATE_ERROR_RECOVERY;	}
				}
			break;

		case CLI_STATE_COMMUNICATE:
				/* send data */
			snprintf(buf, sizeof(buf),
					"{\"type\":\"data\",\"id\":\"%s\",\"timestamp\":\"20%02d-%02d-%02dT%02d:%02d:%02dZ\",\"acceleration\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f},\"status\":\"normal\"}",
					DEVICE_ID, high_scores[0].t.year, high_scores[0].t.month,  high_scores[0].t.day,  high_scores[0].t.hour,  high_scores[0].t.min,  high_scores[0].t.sec,
					high_scores[0].x, high_scores[0].y, high_scores[0].z);
			if (send(sock, buf, strlen(buf), 0) < 0) {
				appLog("[CLI] Send error \r\n");
				state = CLI_STATE_ERROR_RECOVERY;
				break;	}
				/* Non-blocking Receive */
			fds[0].fd = sock;
			fds[0].events = POLLIN;
			ret = poll(fds, 1, CONN_TIMEOUT_MS);
			if(ret > 0){
				if(fds[0].revents & POLLIN){
					int len = recv(sock, buf, sizeof(buf)-1, 0);
					if(len > 0){
						buf[len] = '\0';
						msgProcess(buf);	}
					else		appLog("[CLI] Peer closed\r\n");	}
				}
			else if(ret == 0)		appLog("[CLI] Rx Timeout\r\n");
			else		appLog("[CLI] Poll error (Rx)\r\n");
			state = CLI_STATE_ERROR_RECOVERY;			// close after one transaction

		case CLI_STATE_ERROR_RECOVERY:
			if (sock >= 0) {
				close(sock);
				sock = -1;	}
			state = CLI_STATE_DISCONNECTED;
			osDelay(RETRY_INTERVAL);
			break;
    	}
  	}
	}

/**
* @brief Function implementing the serverTask thread.
* @param argument: Not used
* @retval None
*/
void StartServerTask(void const * argument){
	int listen_sock = -1;
	int client_sock = -1;
	struct sockaddr_in serv_addr = {0};
	struct sockaddr_in client_addr = {0};
	static char buf[NET_BUF_SIZE];
	srvState_t state = SRV_STATE_OFFLINE;
	appLog("[SRV] Server Task Online\r\n");

  while(1){
		switch(state){
		case SRV_STATE_OFFLINE:
			waitForNetworkAvailable();
			listen_sock = socket(AF_INET, SOCK_STREAM, 0);
			if(listen_sock < 0){
				appLog("[SRV] Socket create failed\r\n");
				state = SRV_STATE_ERROR_RECOVERY;
				break;	}
				/* Allow immediate port reuse */
			int opt = 1;
			setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
			fcntl(listen_sock, F_SETFL, fcntl(listen_sock, F_GETFL, 0) | O_NONBLOCK);			// Set non-blocking
      serv_addr.sin_family = AF_INET;
      serv_addr.sin_addr.s_addr = INADDR_ANY;
      serv_addr.sin_port = htons(TCP_PORT);

      if(bind(listen_sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0){
      	appLog("[SRV] Bind Failed\r\n");
      	state = SRV_STATE_ERROR_RECOVERY;
      	break;	}
      if(listen(listen_sock, 1) < 0){
      	appLog("[SRV] Listen Failed\r\n");
      	state = SRV_STATE_ERROR_RECOVERY;
      	break;	}
      appLog("[SRV] Listening on port %d\r\n", TCP_PORT);
      state = SRV_STATE_LISTENING;
      break;

		case SRV_STATE_LISTENING:
				/* Check both RUNNING and LINK_UP bits to recycle socket on pause/link down */
			EventBits_t mask = SYS_BIT_RUNNING | SYS_BIT_LINK_UP;
			if((xEventGroupGetBits(sysEventGroup) & mask) != mask){
				appLog("[SRV] Recycling socket\r\n");
				state = SRV_STATE_ERROR_RECOVERY;
				break;	}
      fd_set read_fds;
      struct timeval timeout = { .tv_usec = CONN_TIMEOUT_US };
      FD_ZERO(&read_fds);
      FD_SET(listen_sock, &read_fds);

      if(select(listen_sock + 1, &read_fds, NULL, NULL, &timeout) > 0){
      	socklen_t addr_len = sizeof(client_addr);
      	client_sock = accept(listen_sock, (struct sockaddr*)&client_addr, &addr_len);
      	if(client_sock >= 0){
      		char peer_ip[16];
      		inet_ntop(AF_INET, &client_addr.sin_addr, peer_ip, sizeof(peer_ip));
      			/* Set peer socket to non-blocking for communication */
      		fcntl(client_sock, F_SETFL, fcntl(client_sock, F_GETFL, 0) | O_NONBLOCK);
      		appLog("[SRV] Peer connected: %s\r\n", peer_ip);
      		state = SRV_STATE_COMMUNICATE;	}
      	}
      break;

		case SRV_STATE_COMMUNICATE:
			fd_set comm_fds;
			struct timeval comm_timeout = { .tv_usec = CONN_TIMEOUT_US };
			FD_ZERO(&comm_fds);
			FD_SET(client_sock, &comm_fds);

			if(select(client_sock + 1, &comm_fds, NULL, NULL, &comm_timeout) > 0){
				int len = recv(client_sock, buf, sizeof(buf) - 1, 0);
				if(len > 0){
					msgProcess(buf);			// Formward to processing task

						/* Reply */
					const char* reply = "{\"type\": \"okay\"}";
					send(client_sock, reply, strlen(reply), 0);	}
				if(len <= 0){
					close(client_sock);
					client_sock = -1;
					state = SRV_STATE_LISTENING;
					break;	}
				}
			else{
        appLog("[SRV] Comm Timeout/Idle\r\n");
        close(client_sock);
        client_sock = -1;
        state = SRV_STATE_LISTENING;
				}
			break;

		case SRV_STATE_ERROR_RECOVERY:
      if(listen_sock >= 0){
      	close(listen_sock);
      	listen_sock = -1; }
      if(client_sock >= 0){
      	close(client_sock);
      	client_sock = -1;	}
      osDelay(RETRY_INTERVAL);
      state = SRV_STATE_OFFLINE;
      break;
			}
  	}
	}

/**
* @brief Function implementing the UDPHeartbeat thread.
* @param argument: Not used
* @retval None
*/
void StartUDPHeartbeat(void const * argument){
	int sock = -1;
	struct sockaddr_in broadcast_addr = {0};
	static char json_buf[NET_BUF_SIZE];
	int broadcast_enable = 1;
	rtc_time_t t;

		/* Configure target broadcast address and port */
	broadcast_addr.sin_family = AF_INET;
	broadcast_addr.sin_port = htons(BROADCAST_PORT);
	broadcast_addr.sin_addr.s_addr = IPADDR_BROADCAST;	// 255.255.255.255
	udpState_t state = UDP_STATE_OFFLINE;
  uint32_t previousWakeTime;
  appLog("[UDP] UDP heartbeat service ready\r\n");

	while(1){
		switch(state){
		case UDP_STATE_OFFLINE:
			waitForNetworkAvailable();
			sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
			if(sock < 0){
				state = UDP_STATE_ERROR_RECOVERY;
				appLog("[UDP] Socket create fail, retrying...\r\n");
				break;	}
			if(setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast_enable, sizeof(broadcast_enable)) < 0){
				state = UDP_STATE_ERROR_RECOVERY;
				appLog("[UDP] setsockopt fail, retrying...\r\n");
				break;	}
			appLog("[UDP] UDP heartbeat service online\r\n");
			state = UDP_STATE_BROADCASTING;
			previousWakeTime = osKernelSysTick();

		case UDP_STATE_BROADCASTING:
				/* Check both RUNNING and LINK_UP bits to recycle socket on pause/link down */
			EventBits_t mask = SYS_BIT_RUNNING | SYS_BIT_LINK_UP;
			if((xEventGroupGetBits(sysEventGroup) & mask) != mask){
				appLog("[UDP] Recycling socket\r\n");
				state = UDP_STATE_ERROR_RECOVERY;
				break;	}

			RTC_GetTime(&t);
			snprintf(json_buf, sizeof(json_buf),
					"{\"type\": \"presence\",\"id\": \"%s\",\"ip\": \"%s\",\"timestamp\": \"20%02d-%02d-%02dT%02d:%02d:%02dZ\"}",
					DEVICE_ID, DEVICE_IP, t.year, t.month, t.day, t.hour, t.min, t.sec);
			if(sendto(sock, json_buf, (size_t)strlen(json_buf), 0, (struct sockaddr*)&broadcast_addr, sizeof(broadcast_addr)) < 0){
				appLog("[UDP] Send error - recycling socket\r\n");
				state = UDP_STATE_ERROR_RECOVERY;
				break;	}
			osDelayUntil(&previousWakeTime, UDP_HEARTBEAT_INTERVAL);
			break;

		case UDP_STATE_ERROR_RECOVERY:
			if (sock >= 0) {
				close(sock);
				sock = -1;	}
			state = UDP_STATE_OFFLINE;
			osDelay(RETRY_INTERVAL);
			break;
			}
		}
	}

void StartBroadcastListenerTask(void const * argument){
	int sock = -1;
	struct sockaddr_in bind_addr = {0};
	struct sockaddr_in rx_addr;
	static char buf[NET_BUF_SIZE];
	struct pollfd fds[1];
	int opt_val = 1;

	bind_addr.sin_family = AF_INET;
	bind_addr.sin_port = htons(BROADCAST_PORT);
	bind_addr.sin_addr.s_addr = INADDR_ANY;
	lstState_t state = LST_STATE_OFFLINE;
	appLog("[LST] Broadcast Listener Started\r\n");

	while(1){
		switch(state){
		case LST_STATE_OFFLINE:
			waitForNetworkAvailable();
				/* Create socket */
			sock = socket(AF_INET, SOCK_DGRAM, 0);
			if(sock < 0){
				appLog("[LST] Socket create failed\r\n");
				state = LST_STATE_ERROR_RECOVERY;
				break;	}
			if(setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &opt_val, sizeof(opt_val)) < 0)
				appLog("[LST] Warning: Failed to set SO_BROADCAST\r\n");
				/* Bind to broadcast port */
			if(bind(sock, (struct sockaddr*)&bind_addr, sizeof(bind_addr)) < 0){
				appLog("[LST] Socket bind failed\r\n");
				state = LST_STATE_ERROR_RECOVERY;
				break;	}
			fcntl(sock, F_SETFL, fcntl(sock, F_GETFL, 0) | O_NONBLOCK);			// Set socket to non-blocking
				/* Setup poll structure */
			fds[0].fd = sock;
			fds[0].events = POLLIN;
			appLog("[LST] Listening on UDP %lu\r\n", BROADCAST_PORT);
			state = LST_STATE_LISTENING;

		case LST_STATE_LISTENING:
				/* Check both RUNNING and LINK_UP bits to recycle socket on pause/link down */
			EventBits_t mask = SYS_BIT_RUNNING | SYS_BIT_LINK_UP;
			if((xEventGroupGetBits(sysEventGroup) & mask) != mask){
				appLog("[LST] Recycling socket\r\n");
				state = LST_STATE_ERROR_RECOVERY;
				break;	}
				/* Poll with timeout */
			int ret = poll(fds, 1, CONN_TIMEOUT_MS);
			if(ret < 0){
				appLog("[LST] Poll error\r\n");
				state = LST_STATE_ERROR_RECOVERY;
				break;	}
			if(ret == 0)		break;
			if(fds[0].revents & POLLIN){
				socklen_t addr_len = sizeof(rx_addr);
				int len = recvfrom(sock, buf, sizeof(buf)-1, 0, (struct sockaddr*)&rx_addr, &addr_len);
				if(len > 0){
					buf[len] = '\0';
					state = LST_STATE_PROCESSING;	}
				else		break;	}

		case LST_STATE_PROCESSING:
			msgProcess(buf);			// Forward to processing task
				/* Go back to listening */
			state = LST_STATE_LISTENING;
			break;

		case LST_STATE_ERROR_RECOVERY:
			if (sock >= 0) {
				close(sock);
				sock = -1;	}
      osDelay(RETRY_INTERVAL);
      state = LST_STATE_OFFLINE;
			break;
			}
		}
	}

void prune_dead_peers(void){
	osMutexWait(ipConfigMutexHandle, osWaitForever);
	uint32_t now = HAL_GetTick();
	for(int i = 0; i < ip_count; ){
		if((now - peerList[i].last_seen) > PEER_TIMEOUT_MS){
			appLog("[TTL] Peer timeout (pruned): %s\r\n", peerList[i].ip);
			for(int j = i; j < ip_count-1; j++)		peerList[j] = peerList[j+1];			// Shift remaining elements down
			memset(&peerList[ip_count-1], 0, sizeof(peer_t));			// Clear last element
			ip_count--;	}
		else		i++;	}
	osMutexRelease(ipConfigMutexHandle);
	}

void StartNtpSyncTask(void const* argument){
	int sock = -1;
	struct sockaddr_in server_addr;
	struct hostent* server_host;
	uint8_t ntp_packet[NTP_PACKET_SIZE];
	appLog("[NTP] Service Started. Waiting for Link...\r\n");
  while (netif_default == NULL || !netif_is_link_up(netif_default))		osDelay(100);
  // 3. DNS Configuration: Force 8.8.8.8 directly
  ip_addr_t dns_addr;
  IP_ADDR4(&dns_addr, 8, 8, 8, 8);
  dns_setserver(0, &dns_addr);
	appLog("[NTP] Resolving %s...\r\n", NTP_SERVER_NAME);

	server_host = gethostbyname(NTP_SERVER_NAME);
	if(server_host != NULL){
		sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
		if(sock >= 0){
			memset(ntp_packet, 0, NTP_PACKET_SIZE);
			ntp_packet[0] = 0x1B;			// LI=0, VN=3, Mode=3

			server_addr.sin_family = AF_INET;
			server_addr.sin_port = htons(NTP_PORT);
			memcpy(&server_addr.sin_addr.s_addr, server_host->h_addr, server_host->h_length);

			struct timeval tv = { .tv_usec = CONN_TIMEOUT_US };
			setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
			if(sendto(sock, ntp_packet, NTP_PACKET_SIZE, 0, (struct sockaddr*)&server_addr, sizeof(server_addr)) >= 0){
				socklen_t addr_len = sizeof(server_addr);
				int len = recvfrom(sock, ntp_packet, NTP_PACKET_SIZE, 0, (struct sockaddr*)&server_addr, &addr_len);
				if(len > 0){
					uint32_t seconds = (ntp_packet[40] << 24) | (ntp_packet[41] << 16) | (ntp_packet[42] << 8) | ntp_packet[43];
					uint32_t unix_time = seconds - NTP_TIMESTAMP_DELTA;
					unix_time += (TIMEZONE_OFFSET_HRS * 3600);

					struct tm* time_info;
					time_t raw_time = (time_t)unix_time;
					time_info = gmtime(&raw_time);

					rtc_time_t new_rtc;
          new_rtc.sec   = time_info->tm_sec;
          new_rtc.min   = time_info->tm_min;
          new_rtc.hour  = time_info->tm_hour;
          new_rtc.day   = time_info->tm_mday;
          new_rtc.month = time_info->tm_mon + 1;
          new_rtc.year  = (time_info->tm_year + 1900) % 100;
          new_rtc.wday  = time_info->tm_wday + 1;

          RTC_WriteTime(&new_rtc);
          appLog("[NTP] Sync Complete.\r\n");	}
				else		appLog("[NTP] Response Timeout.\r\n");	}
			close(sock);	}
		else appLog("[NTP] Socket creation failed.\r\n");	}
	else		appLog("[NTP] DNS Resolution Failed.\r\n");

	xEventGroupSetBits(sysEventGroup, SYS_BIT_TIME_SYNC);
	vTaskDelete(NULL);
	}

