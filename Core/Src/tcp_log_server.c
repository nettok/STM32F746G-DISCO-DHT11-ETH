/*
 * tcp_log_server.c
 *
 *  Created on: Apr 26, 2020
 *      Author: nettok
 */


#include "tcp_log_server.h"


uint8_t tcpLogMessageStorageBuffer[TCP_LOG_MESSAGE_QUEUE_LENGTH * sizeof(TcpLogMessage)];

StaticQueue_t tcpLogMessageStaticQueue;

QueueHandle_t tcpLogMessageQueue;

osThreadId_t tcpLogServerTaskHandle;
const osThreadAttr_t tcpLogServerTask_attributes = {
  .name = "tcpLogServerTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = TCP_LOG_MESSAGE_SERVER_TREAD_STACK_SIZE * 4
};

uint8_t tcp_log_client_connected = 0;

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
          tcp_log_client_connected = 1;

          // We will only accept one connection at a time
          while(1)
          {
            tcpLogMessageQueueReceiveStatus = xQueueReceive(tcpLogMessageQueue, &tcpLogMessage, 0);
            if (tcpLogMessageQueueReceiveStatus == pdPASS)
            {
              write_err = netconn_write(newconn, tcpLogMessage.Buf, tcpLogMessage.len, NETCONN_NOCOPY);
              if (write_err != ERR_OK)
              {
                break;
              }
            }
          }

          tcp_log_client_connected = 0;
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

void log_to_tcp(TcpLogMessage *message)
{
  if (tcp_log_client_connected)
  {
    xQueueSend(tcpLogMessageQueue, message, 0);
  }
}
