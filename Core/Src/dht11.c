/*
 * dht11.c
 *
 *  Created on: May 1, 2020
 *      Author: nettok
 */


#include "dht11.h"


static GPIO_InitTypeDef GPIO_DHT11_DATA_InitStruct = { 0 };

static void setDHT11Mode(uint32_t mode) {
  GPIO_DHT11_DATA_InitStruct.Pin = DHT11_DATA_Pin;
  GPIO_DHT11_DATA_InitStruct.Mode = mode;
  GPIO_DHT11_DATA_InitStruct.Pull = GPIO_NOPULL;
  GPIO_DHT11_DATA_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT11_DATA_GPIO_Port, &GPIO_DHT11_DATA_InitStruct);
}

static void setDHT11ToOutputMode() {
  setDHT11Mode(GPIO_MODE_OUTPUT_PP);
}

static void setDHT11ToInputMode() {
  setDHT11Mode(GPIO_MODE_INPUT);
}

static void setDHT11Pin(GPIO_PinState pinState) {
  HAL_GPIO_WritePin(DHT11_DATA_GPIO_Port, DHT11_DATA_Pin, pinState);
}

static GPIO_PinState getDHT11Pin() {
  return HAL_GPIO_ReadPin(DHT11_DATA_GPIO_Port, DHT11_DATA_Pin);
}

static uint32_t waitForDHT11PullUp() {
  uint32_t downCycles = 0;
  if (getDHT11Pin() == 1) {
    while (getDHT11Pin() == 1) { // wait until down first
    }
  }
  while (getDHT11Pin() == 0) { // wait for pull up
    downCycles++;
  }
  return downCycles;
}

static uint32_t waitForDHT11PullDown() {
  uint32_t upCycles = 0;
  if (getDHT11Pin() == 0) {
    while (getDHT11Pin() == 0) { // wait until up first
    }
  }
  while (getDHT11Pin() == 1) { // wait for pull down
    upCycles++;
  }
  return upCycles;
}

static uint8_t readData(uint32_t thresholdCycles) {
  uint8_t i, j;
  for (j = 0; j < 8; j++) {
    waitForDHT11PullUp();
    uint32_t upCycles = waitForDHT11PullDown();
    if (upCycles > thresholdCycles) {
      i |= (1 << (7 - j));    // bit is 1
    } else {
      i &= ~(1 << (7 - j));  // bit is 0
    }
  }
  return i;
}

uint8_t dht11_read_climate(Climate *climate) {
  uint8_t integralHumidityData;
  uint8_t decimalHumidityData;
  uint8_t integralTempData;
  uint8_t decimalTempData;
  uint8_t checkSum;

  // initial state for DHT11
  setDHT11ToInputMode();
  osDelay(1000); // to pass unstable status of the DHT11 sensor

  taskENTER_CRITICAL();

  // start communication
  setDHT11ToOutputMode();
  setDHT11Pin(0);
  HAL_Delay(18);
  setDHT11Pin(1);

  // wait for DHT11 response
  setDHT11ToInputMode();
  waitForDHT11PullUp();
  uint32_t upCycles = waitForDHT11PullDown();

  // upCycles of last pull-down is equivalent to 80 microseconds
  // Use this to interpolate data transmission bit responses knowing that:
  //   - bit 0: 26-28 microseconds
  //   - bit 1: 70 microseconds
  //
  uint32_t thresholdCycles = (upCycles / 2) + (10 * upCycles / 80); // ~50 microseconds

  // start sensor data transmission
  integralHumidityData = readData(thresholdCycles);
  decimalHumidityData = readData(thresholdCycles);
  integralTempData = readData(thresholdCycles);
  decimalTempData = readData(thresholdCycles);
  checkSum = readData(thresholdCycles);

  taskEXIT_CRITICAL();

  uint8_t dataSum = integralHumidityData + decimalHumidityData
                    + integralTempData + decimalTempData;

  if (checkSum != dataSum) {
    return 0;
  }

  climate->integralTemp = integralTempData;
  climate->decimalTemp = decimalTempData;
  climate->integralHumidity = integralHumidityData;
  climate->decimalHumidity = decimalHumidityData;

  return 1;
}
