/*
 * tcp_log_server.h
 *
 *  Created on: Apr 26, 2020
 *      Author: nettok
 */

#ifndef INC_TCP_LOG_SERVER_H_
#define INC_TCP_LOG_SERVER_H_

#include "cmsis_os.h"
#include "lwip.h"
#include "queue.h"
#include "api.h"

#define TCP_LOG_MESSAGE_QUEUE_LENGTH 16
#define TCP_LOG_MESSAGE_SERVER_TREAD_STACK_SIZE 512
#define TCP_LOG_MESSAGE_SERVER_PORT 7777
#define TCP_LOG_MESSAGE_BUFFER_LENGTH 127

typedef struct {
  uint8_t len;
  uint8_t Buf[TCP_LOG_MESSAGE_BUFFER_LENGTH];
} TcpLogMessage;

void tcp_log_server_init();
void log_to_tcp(TcpLogMessage *message);

#endif /* INC_TCP_LOG_SERVER_H_ */
