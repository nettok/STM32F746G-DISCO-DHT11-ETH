/*
 * mqtt_event_publisher.h
 *
 *  Created on: Apr 28, 2020
 *      Author: nettok
 */

#ifndef INC_MQTT_EVENT_PUBLISHER_H_
#define INC_MQTT_EVENT_PUBLISHER_H_

#include "cmsis_os.h"
#include "queue.h"
#include "api.h"
#include "string.h"
#include "nx_mqtt.h"

#define MQTT_EVENT_PUBLISHER_BROKER_HOST_IP "192.168.0.31"
#define MQTT_EVENT_PUBLISHER_BROKER_PORT 1883

#define MQTT_EVENT_PUBLISHER_CLIENT_ID "STM32F746GDISCO"

#define MQTT_EVENT_PUBLISHER_THREAD_STACK_SIZE 1024
#define MQTT_EVENT_PUBLISHER_QUEUE_LENGTH 32

#define MQTT_EVENT_PUBLISHER_TOPIC_BUFFER_LENGTH 127
#define MQTT_EVENT_PUBLISHER_PAYLOAD_BUFFER_LENGTH 127

typedef struct {
  char topic[MQTT_EVENT_PUBLISHER_TOPIC_BUFFER_LENGTH];
  uint8_t payload[MQTT_EVENT_PUBLISHER_PAYLOAD_BUFFER_LENGTH];
  uint8_t payload_length;
} MqttEvent;

void mqtt_evt_pub_init();
void mqtt_evt_pub_publish_event(MqttEvent *event);
void mqtt_evt_pub_publish(char *topic, char *payload);

#endif /* INC_MQTT_EVENT_PUBLISHER_H_ */
