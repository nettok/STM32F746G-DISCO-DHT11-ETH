/*
 * sntp_rtc_bridge.h
 *
 *  Created on: Apr 26, 2020
 *      Author: nettok
 */

#ifndef INC_SNTP_RTC_BRIDGE_H_
#define INC_SNTP_RTC_BRIDGE_H_

#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_rtc.h"
#include "time.h"

#define SNTP_SET_SYSTEM_TIME(sec) sntp_set_system_time(sec);

void sntp_rtc_bridge_init(RTC_HandleTypeDef hrtc);
void sntp_set_system_time(uint32_t sec);

#endif /* INC_SNTP_RTC_BRIDGE_H_ */
