/*
 * tcp_log_server.c
 *
 *  Created on: Apr 26, 2020
 *      Author: nettok
 */


#include "tcp_log_server.h"


static uint8_t tcpLogMessageStorageBuffer[TCP_LOG_MESSAGE_QUEUE_LENGTH * sizeof(TcpLogMessage)];

static StaticQueue_t tcpLogMessageStaticQueue;

static QueueHandle_t tcpLogMessageQueue = NULL;

static osThreadId_t tcpLogServerTaskHandle;
static const osThreadAttr_t tcpLogServerTask_attributes = {
  .name = "tcpLogServerTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = TCP_LOG_MESSAGE_SERVER_TREAD_STACK_SIZE * 4
};

static void TcpLogServerTask(void *argument)
{
  struct netconn *conn, *newconn;

  err_t err, accept_err, write_err;

  TcpLogMessage tcpLogMessage;
  BaseType_t tcpLogMessageQueueReceiveStatus;

  LWIP_UNUSED_ARG(argument);

  conn = netconn_new(NETCONN_TCP);
  if (conn!= NULL)
  {
    err = netconn_bind(conn, IP_ADDR_ANY, TCP_LOG_MESSAGE_SERVER_PORT);
    if (err == ERR_OK)
    {
      netconn_listen(conn);

      while (1)
      {
        accept_err = netconn_accept(conn, &newconn);

        if(accept_err == ERR_OK)
        {
          // We will only accept one connection at a time
          while(1)
          {
            tcpLogMessageQueueReceiveStatus = xQueueReceive(tcpLogMessageQueue, &tcpLogMessage, portMAX_DELAY);
            if (tcpLogMessageQueueReceiveStatus == pdPASS)
            {
              write_err = netconn_write(newconn, tcpLogMessage.message, tcpLogMessage.length, NETCONN_COPY);
              if (write_err != ERR_OK)
              {
                break;
              }
            }
          }

          netconn_delete(newconn);
        }
      }
    }
  }
}

void tcp_log_server_init()
{
  tcpLogMessageQueue = xQueueCreateStatic(TCP_LOG_MESSAGE_QUEUE_LENGTH, sizeof(TcpLogMessage), tcpLogMessageStorageBuffer, &tcpLogMessageStaticQueue);
  if (tcpLogMessageQueue != NULL)
  {
    tcpLogServerTaskHandle = osThreadNew(TcpLogServerTask, NULL, &tcpLogServerTask_attributes);
  }
}

void tcp_log_message(TcpLogMessage *message)
{
  if (tcpLogMessageQueue != NULL && uxQueueSpacesAvailable(tcpLogMessageQueue) > 0)
  {
    xQueueSend(tcpLogMessageQueue, message, 0);
  }
}

void tcp_log_msg(char *msg)
{
  TcpLogMessage tcpLopMessage;
  strncpy(tcpLopMessage.message, msg, TCP_LOG_MESSAGE_BUFFER_LENGTH - 2);
  tcpLopMessage.length = strlen(tcpLopMessage.message);
  tcpLopMessage.message[tcpLopMessage.length] = '\r';
  tcpLopMessage.message[tcpLopMessage.length + 1] = '\n';
  tcpLopMessage.length += 2;
  tcp_log_message(&tcpLopMessage);
}
