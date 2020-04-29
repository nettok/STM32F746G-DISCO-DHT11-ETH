/*
 * mqtt_event_publisher.c
 *
 *  Created on: Apr 28, 2020
 *      Author: nettok
 */


#include "mqtt_event_publisher.h"

static uint8_t mqttEvtStorageBuffer[MQTT_EVENT_PUBLISHER_QUEUE_LENGTH * sizeof(MqttEvent)];
static StaticQueue_t mqttEvtStaticQueue;
static QueueHandle_t mqttEvtQueue = NULL;

static osThreadId_t mqttEvtPubTaskHandle;
static const osThreadAttr_t mqttEvtPubTask_attributes = {
  .name = "mqttEvtPubTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = MQTT_EVENT_PUBLISHER_TREAD_STACK_SIZE * 4
};

static ip4_addr_t ip_addr;

static void MqttEventPublisherTask(void *argument)
{
  struct netconn *conn;

  err_t err, conn_err, write_err;

  MqttEvent event;
  BaseType_t eventQueueReceiveStatus;

  LWIP_UNUSED_ARG(argument);

  while(1)
  {
    conn = netconn_new(NETCONN_TCP);
    if (conn != NULL)
    {
      err = netconn_bind(conn, IP_ADDR_ANY, 0);
      if (err == ERR_OK)
      {
        conn_err = netconn_connect(conn, &ip_addr, MQTT_EVENT_PUBLISHER_BROKER_PORT);
        if (conn_err == ERR_OK)
        {
          while(1)
          {
            eventQueueReceiveStatus = xQueueReceive(mqttEvtQueue, &event, portMAX_DELAY);
            if (eventQueueReceiveStatus == pdPASS)
            {
              write_err = netconn_write(conn, event.payload, event.payload_length, NETCONN_COPY);
              if (write_err != ERR_OK)
              {
                break;
              }
            }
          }
        }
      }
    }
    netconn_delete(conn); // Clean up
    osDelay(3000); // Try to reconnect after some time
  }
}



void mqtt_evt_pub_init()
{
  mqttEvtQueue = xQueueCreateStatic(MQTT_EVENT_PUBLISHER_QUEUE_LENGTH, sizeof(MqttEvent), mqttEvtStorageBuffer, &mqttEvtStaticQueue);
  if (mqttEvtQueue != NULL)
  {
    ip4addr_aton(MQTT_EVENT_PUBLISHER_BROKER_HOST_IP, &ip_addr);

    mqttEvtPubTaskHandle = osThreadNew(MqttEventPublisherTask, NULL, &mqttEvtPubTask_attributes);
  }
}

void mqtt_evt_pub_publish_event(MqttEvent *event)
{
  if (mqttEvtQueue != NULL && uxQueueSpacesAvailable(mqttEvtQueue) > 0)
  {
    xQueueSend(mqttEvtQueue, event, 0);
  }
}

void mqtt_evt_pub_publish(char *topic, char *payload)
{
  MqttEvent event;
  strncpy(event.topic, topic, MQTT_EVENT_PUBLISHER_TOPIC_BUFFER_LENGTH);
  strncpy((char*)event.payload, payload, MQTT_EVENT_PUBLISHER_PAYLOAD_BUFFER_LENGTH);
  event.payload_length = strlen((char*)event.payload);
  mqtt_evt_pub_publish_event(&event);
}
