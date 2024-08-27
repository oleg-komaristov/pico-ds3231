/*
 * Copyright (c) 2023 Antonio González
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef DS3231_H
#define DS3231_H

#include <stdio.h>
#include <time.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

/** \file ds3231.h
 * \brief Library for using a DS3231 real-time clock with the Raspberry
 * Pi Pico
 *
*/

#ifndef DS3231_I2C_ADDRESS
	#define DS3231_I2C_ADDRESS 0x68
#endif
#ifndef DS3231_I2C_FREQ
	#define DS3231_I2C_FREQ 100 * 1000
#endif

enum rtc_register {
    DS3231_DATETIME_REG = 0x00u,   
    DS3231_ALARM1_REG = 0x07u,    
    DS3231_ALARM2_REG = 0x0Bu,
    DS3231_CONTROL_REG = 0x0Eu,
    DS3231_STATUS_REG = 0x0Fu,
    DS3231_AGING_OFFSET_REG = 0x10u,
    DS3231_TEMPERATURE_REG = 0x11u
};

typedef struct ds3231_rtc {
    i2c_inst_t *i2c_port;
    uint8_t i2c_addr;
    uint8_t i2c_sda_pin;
    uint8_t i2c_scl_pin;
} ds3231_rtc_t;

typedef enum {
    DS3231_ACTIVE_TIME   = 1 << 0,
    DS3231_ACTIVE_SQUARE = 1 << 1
} ds3231_active_t;

typedef enum  {
    DS3231_AL1_EVERY_SECOND   = 0xF,
    DS3231_AL1_MATCH_SECONDS  = 0xE,
    DS3231_AL1_MATCH_M_S      = 0xC,
    DS3231_AL1_MATCH_H_M_S    = 0x8,
    DS3231_AL1_MATCH_DT_H_M_S = 0x0,
    DS3231_AL1_MATCH_DY_H_M_S = 0x10
} ds3231_alarm1_mode_t;

typedef enum {
    DS3231_AL2_EVERY_MINUTE   = 0xE,
    DS3231_AL2_MATCH_M        = 0xC,
    DS3231_AL2_MATCH_H_M      = 0x8,
    DS3231_AL2_MATCH_DT_H_M   = 0x0,
    DS3231_AL2_MATCH_DY_H_M   = 0x10
} ds3231_alarm2_mode_t;

typedef enum {
    DS3231_ALARM1,
    DS3231_ALARM2
} ds3231_alarm_t;

typedef enum {
    DS3231_SQW_MODE_SIGNAL,
    DS3231_SQW_MODE_ALARM
} ds3231_sqw_mode_t;

typedef struct {
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    union {
        ds3231_alarm1_mode_t alarm1;
        ds3231_alarm2_mode_t alarm2;
    } mode;
} ds3231_alarm_time_t;

/*! \brief Initialise a DS321 real-time clock
 *
 * \param rtc Pointer to a ds3231_rtc_t structure
 * \param i2c_port The I2C instance, i2c0 or i2c1
 * \param i2c_addr 7-bit address of the DS3231 device
 */
void ds3231_init(ds3231_rtc_t *rtc, i2c_inst_t *i2c_port,
                 uint8_t i2c_sda_pin, uint8_t i2c_scl_pin);

/*! \brief Sets components active on battery power.
 * 
 * \param active Active components.
 */
void ds3231_set_on_battery(ds3231_rtc_t *rtc, ds3231_active_t active);

/*! \brief Reads the current SQW output mode.
 * 
 * \param rtc Pointer to a ds3231_rtc_t structure
 * \return Current mode of SQW pin.
 */
ds3231_sqw_mode_t ds3231_get_sqw_mode(ds3231_rtc_t *rtc);

/*! \brief Sets the SQW output mode.
 * 
 * \param rtc Pointer to a ds3231_rtc_t structure
 * \param mode SQW pin mode.
 */
void ds3231_set_sqw_mode(ds3231_rtc_t *rtc, ds3231_sqw_mode_t mode);

/*! \brief Set date time to RTC.
 *
 * \note This method resets the OSF bit.
 * 
 * \param rtc Pointer to a ds3231_rtc_t structure
 * \param dt date to set
 */
void ds3231_set_datetime(ds3231_rtc_t *rtc, const struct tm *dt);

void ds3231_set_time(ds3231_rtc_t *rtc, const time_t time);

void ds3231_get_datetime(ds3231_rtc_t *rtc, struct tm *dt);

time_t ds3231_get_time(ds3231_rtc_t *rtc);

/*! \brief Format a ds3231_datetime_t structure into a ISO 8601 string
 * 
 * Date/time string in the format YYYY-MM-DDTHH:MM:SS
 *
 * \param buf Character buffer to accept generated string
 * \param buf_size The size of the passed-in buffer (at least 19)
 * \param dt The datetime to be converted
 */
void ds3231_isoformat(char *buf, uint8_t buf_size, const struct tm *dt);

/*! \brief Format ds3231_datetime_t structure into a date string
 * 
 * \param buf Character buffer to accept generated string
 * \param buf_size The size of the passed-in buffer (at least 10)
 * \param dt The datetime to be converted
 */
void ds3231_str_date(char *buf, uint8_t buf_size, const struct tm *dt);

/*! \brief Format ds3231_datetime_t structure into a time string
 * 
 * \param buf Character buffer to accept generated string
 * \param buf_size The size of the passed-in buffer (at least 8)
 * \param dt The datetime to be converted
 */
void ds3231_str_time(char *buf, uint8_t buf_size, const struct tm *dt);

/*! \brief Format ds3231_datetime_t structure into a ctime string
 * 
 * Convert datetime to a string in the format "Www Mmm dd hh:mm:ss
 * yyyy", where Www is the weekday, Mmm the month in letters, dd the day
 * of the month, hh:mm:ss the time, and yyyy the year.
 * 
 * \param buf Character buffer to accept generated string
 * \param buf_size The size of the passed-in buffer (at least 25)
 * \param dt The datetime to be converted
 */
void ds3231_ctime(char *buf, uint8_t buf_size, const struct tm *dt);

/*! \brief Check if the oscillator has stopped
 * 
 * \param rtc Pointer to the ds3231_rtc_t structure
 */
bool ds3231_oscillator_is_stopped(ds3231_rtc_t *rtc);

/*! \brief Read the temperature
 * 
 * \param rtc Pointer to the ds3231_rtc_t structure
 * 
 * Note that the temperature registers are updated every 64 seconds. It
 * thus makes little sense to read the temperature register more often
 * than this.
 * 
 * \return Temperature in Celsius degrees.
 */
float ds3231_get_temperature(ds3231_rtc_t *rtc);

void ds3231_set_alarm(ds3231_rtc_t *rtc, ds3231_alarm_t alarm, const ds3231_alarm_time_t *time, _Bool active);

ds3231_alarm_time_t ds3231_get_alarm_time(ds3231_rtc_t *rtc, ds3231_alarm_t alarm);

_Bool ds3231_get_alarm_active(ds3231_rtc_t *rtc, ds3231_alarm_t alarm);

void ds3231_set_alarm_active(ds3231_rtc_t *rtc, ds3231_alarm_t alarm, _Bool state);

_Bool ds3231_is_alarm_fired(ds3231_rtc_t *rtc, ds3231_alarm_t alarm);

void ds3231_clear_alarm(ds3231_rtc_t *rtc, ds3231_alarm_t alarm);

#endif
