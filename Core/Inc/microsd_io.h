/*
 * microsd_io.h
 *
 *  Created on: May 1, 2020
 *      Author: nettok
 */

#ifndef INC_MICROSD_IO_H_
#define INC_MICROSD_IO_H_

#include "cmsis_os.h"
#include "queue.h"
#include "string.h"
#include "fatfs.h"

#define MICROSD_IO_THREAD_STACK_SIZE 1024
#define MICROSD_IO_COMMAND_QUEUE_LENGTH 32

#define MICROSD_IO_PATH_BUFFER_LENGTH 127
#define MICROSD_IO_DATA_BUFFER_LENGTH 127

typedef enum
{
  USDIO_WRITE,
} MicroSdIoCommandType;

typedef struct {
  MicroSdIoCommandType commandType;
  char path[MICROSD_IO_PATH_BUFFER_LENGTH];
  uint8_t data[MICROSD_IO_DATA_BUFFER_LENGTH];
  uint8_t data_length;
} MicroSdIoCommand;

void microsd_io_init();
void microsd_io_write(char *path, uint8_t *data, uint8_t data_length);

#endif /* INC_MICROSD_IO_H_ */
