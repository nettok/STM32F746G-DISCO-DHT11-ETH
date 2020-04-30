/*
 * nx_mqtt.c
 *
 *  Created on: Apr 29, 2020
 *      Author: nettok
 */


#include "nx_mqtt.h"


// Re-using the same packet structure buffer is safe within the context of "mqtt_event_publisher"
// as only one event is published at a time.
//
// And also because the netconn_write calls are using NETCONN_COPY.

static uint8_t buffer[256];

static size_t fillConnect(const char *client_id)
{
  size_t client_id_length = strlen(client_id);

  buffer[0] = 1 << 4;                   // fixed_header.control_packet_type
  buffer[1] = 13 + client_id_length;    // fixed_header.remaining_length

  buffer[2] = 0;                        // proto_name_length_msb
  buffer[3] = 4;                        // proto_name_length_lsb
  buffer[4] = 'M';                      // proto_name_char1
  buffer[5] = 'Q';                      // proto_name_char2
  buffer[6] = 'T';                      // proto_name_char3
  buffer[7] = 'T';                      // proto_name_char4

  buffer[8] = 5;                        // protocol_version
  buffer[9] = 2;                        // connect_flags

  buffer[10] = 0;                       // keep_alive_msb
  buffer[11] = 0;                       // keep_alive_lsb

  buffer[12] = 0;                       // properties_length

  buffer[13] = 0;                       // client_id_length_msb
  buffer[14] = client_id_length;        // client_id_length_lsb

  strcpy((char*)&buffer[15], client_id); // client_id
  return 15 + client_id_length;
}

static size_t fillPublish(const char *topic, const void *payload, size_t payload_size)
{
  size_t topic_length = strlen(topic);

  buffer[0] = 3 << 4;                   // fixed_header.control_packet_type
  buffer[1] = 5 + topic_length          // fixed_header.remaining_length
                + payload_size;

  buffer[2] = 0;                        // topic_length_msb
  buffer[3] = topic_length;             // topic_length_lsb

  strcpy((char*)&buffer[4], topic);

  buffer[4 + topic_length] = 0;         // properties_length

  buffer[5 + topic_length] = 0;         // payload_length_msb
  buffer[6 + topic_length] = payload_size; // payload_length_lsb

  memcpy(&buffer[7 + topic_length], payload, payload_size); // payload
  return 7 + topic_length + payload_size;
}

err_t nx_mqtt_connect(struct netconn *conn, const char *client_id)
{
  size_t size = fillConnect(client_id);
  return netconn_write(conn, buffer, size, NETCONN_COPY);
}

err_t nx_mqtt_publish(struct netconn *conn, const char *topic, const void *payload, size_t payload_size)
{
  size_t size = fillPublish(topic, payload, payload_size);
  return netconn_write(conn, buffer, size, NETCONN_COPY);
}
