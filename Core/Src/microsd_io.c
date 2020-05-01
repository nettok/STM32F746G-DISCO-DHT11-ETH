/*
 * microsd_io.c
 *
 *  Created on: May 1, 2020
 *      Author: nettok
 */


#include "microsd_io.h"

static uint8_t microSdIoCommandStorageBuffer[MICROSD_IO_COMMAND_QUEUE_LENGTH * sizeof(MicroSdIoCommand)];
static StaticQueue_t microSdIoCommandStaticQueue;
static QueueHandle_t microSdIoCommandQueue = NULL;

static osThreadId_t microSdIoTaskHandle;
static const osThreadAttr_t microSdIoTask_attributes = {
  .name = "microSdIoTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = MICROSD_IO_THREAD_STACK_SIZE * 4
};

FATFS SDFatFS;  /* File system object for SD card logical drive */
FIL MyFile;     /* File object */
char SDPath[4]; /* SD card logical drive path */

static void write(MicroSdIoCommand *cmd)
{
  FRESULT fRes;
  UINT byteswritten;

  fRes = f_mount(&SDFatFS, (TCHAR const*) SDPath, 0);
  if (fRes == FR_OK)
  {
    fRes = f_open(&MyFile, cmd->path, FA_OPEN_APPEND | FA_WRITE);
    if (fRes == FR_OK)
    {
      f_write(&MyFile, cmd->data, cmd->data_length, &byteswritten);
    }
  }

  f_close(&MyFile);
  f_mount(0, (TCHAR const*) SDPath, 0);
}

static void MicroSdIoTask(void *argument)
{
  MicroSdIoCommand cmd;
  BaseType_t microSdIoCommandReceiveStatus;

  while(1)
  {
    microSdIoCommandReceiveStatus = xQueueReceive(microSdIoCommandQueue, &cmd, portMAX_DELAY);
    if (microSdIoCommandReceiveStatus == pdPASS)
    {
      switch (cmd.commandType)
      {
        case USDIO_WRITE:
          write(&cmd);
      }
    }
  }
}

void microsd_io_init()
{
  microSdIoCommandQueue = xQueueCreateStatic(MICROSD_IO_COMMAND_QUEUE_LENGTH, sizeof(MicroSdIoCommand), microSdIoCommandStorageBuffer, &microSdIoCommandStaticQueue);
  if (microSdIoCommandQueue != NULL)
  {
    microSdIoTaskHandle = osThreadNew(MicroSdIoTask, NULL, &microSdIoTask_attributes);
  }
}

void microsd_io_write(char *path, uint8_t *data, uint8_t data_length)
{
  MicroSdIoCommand cmd;

  if (microSdIoCommandQueue != NULL && uxQueueSpacesAvailable(microSdIoCommandQueue) > 0)
  {
    cmd.commandType = USDIO_WRITE;
    strncpy(cmd.path, path, MICROSD_IO_PATH_BUFFER_LENGTH);
    memcpy(cmd.data, data, data_length);
    cmd.data_length = data_length;

    xQueueSend(microSdIoCommandQueue, &cmd, 0);
  }
}
