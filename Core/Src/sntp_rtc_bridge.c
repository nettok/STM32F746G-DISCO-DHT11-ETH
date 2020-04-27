/*
 * sntp_rtc_bridge.c
 *
 *  Created on: Apr 26, 2020
 *      Author: nettok
 */

#include "sntp_rtc_bridge.h"

static RTC_HandleTypeDef bridgeHrtc;

static RTC_TimeTypeDef rtcTime = {0};
static RTC_DateTypeDef rtcDate = {0};

static struct tm sntpTime;

void sntp_rtc_bridge_init(RTC_HandleTypeDef hrtc)
{
  bridgeHrtc = hrtc;
}

void sntp_set_system_time(uint32_t sec)
{
  int_least64_t sec64 = sec;
  gmtime_r(&sec64, &sntpTime);

  rtcTime.Hours = sntpTime.tm_hour;
  rtcTime.Minutes = sntpTime.tm_min;
  rtcTime.Seconds = sntpTime.tm_sec;
  rtcTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  rtcTime.StoreOperation = RTC_STOREOPERATION_RESET;
  HAL_RTC_SetTime(&bridgeHrtc, &rtcTime, RTC_FORMAT_BIN);

  rtcDate.WeekDay = sntpTime.tm_wday;
  rtcDate.Month = sntpTime.tm_mon + 1;
  rtcDate.Date = sntpTime.tm_mday;
  rtcDate.Year = sntpTime.tm_year - 100;
  HAL_RTC_SetDate(&bridgeHrtc, &rtcDate, RTC_FORMAT_BIN);
}
