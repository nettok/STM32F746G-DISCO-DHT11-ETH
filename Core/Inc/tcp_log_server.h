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
#include "string.h"

#define TCP_LOG_MESSAGE_SERVER_PORT 7777

#define TCP_LOG_MESSAGE_QUEUE_LENGTH 16
#define TCP_LOG_MESSAGE_SERVER_THREAD_STACK_SIZE 512

#define TCP_LOG_MESSAGE_BUFFER_LENGTH 127

typedef struct {
  char message[TCP_LOG_MESSAGE_BUFFER_LENGTH];
  uint8_t length;
} TcpLogMessage;

void tcp_log_server_init();
void tcp_log_message(TcpLogMessage *message);
void tcp_log_msg(char *msg);

#endif /* INC_TCP_LOG_SERVER_H_ */
