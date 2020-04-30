/*
 * nx_mqtt.h
 *
 *  Created on: Apr 29, 2020
 *      Author: nettok
 */

#ifndef INC_NX_MQTT_H_
#define INC_NX_MQTT_H_

#include "api.h"
#include "string.h"

err_t nx_mqtt_connect(struct netconn *conn, const char *client_id);
err_t nx_mqtt_publish(struct netconn *conn, const char *topic, const void *payload, size_t payload_size);

#endif /* INC_NX_MQTT_H_ */
