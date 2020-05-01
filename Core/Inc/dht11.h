/*
 * dht11.h
 *
 *  Created on: May 1, 2020
 *      Author: nettok
 */

#ifndef INC_DHT11_H_
#define INC_DHT11_H_

#include "cmsis_os.h"
#include "main.h"

typedef struct {
        uint8_t integralHumidity;
        uint8_t decimalHumidity;
        uint8_t integralTemp;
        uint8_t decimalTemp;
} Climate;

uint8_t dht11_read_climate(Climate *climate);

#endif /* INC_DHT11_H_ */
