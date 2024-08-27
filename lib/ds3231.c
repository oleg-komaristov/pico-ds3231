/*
 * Copyright (c) 2023 Antonio González
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "ds3231.h"

#define STATUS_OSF (1 << 7)
#define STATUS_AL1 (1 << 0)
#define STATUS_AL2 (1 << 1)

#define DEC2BCD(__value__) ((__value__) / 10 * 16 + ((__value__) % 10))
#define BCD2DEC(__value__) ((__value__) / 16 * 10 + ((__value__) % 16))

static const char *DS3231_WDAYS[7] = {"Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"};

static const char *DS3231_MONTHS[12] = {"Jan", "Feb", "Mar", "Apr",
                                        "May", "Jun", "Jul", "Aug",
                                        "Sep", "Oct", "Nov", "Dec"};

void ds3231_init(ds3231_rtc_t *rtc, i2c_inst_t *i2c_port,
                 uint8_t i2c_sda_pin, uint8_t i2c_scl_pin) {
    rtc->i2c_port = i2c_port;
    rtc->i2c_addr = DS3231_I2C_ADDRESS;
    rtc->i2c_sda_pin = i2c_sda_pin;
    rtc->i2c_scl_pin = i2c_scl_pin;

    i2c_init(i2c_port, DS3231_I2C_FREQ);
    gpio_set_function(i2c_sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(i2c_scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(i2c_sda_pin);
    gpio_pull_up(i2c_scl_pin);

    ds3231_set_on_battery(rtc, DS3231_ACTIVE_TIME);
};

void ds3231_set_on_battery(ds3231_rtc_t *rtc, ds3231_active_t active) {
    uint8_t reg = DS3231_CONTROL_REG;
    uint8_t state;
    i2c_write_blocking(rtc->i2c_port, rtc->i2c_addr, &reg, 1, true);
    i2c_read_blocking(rtc->i2c_port, rtc->i2c_addr, &state, 1, true);
    uint8_t new_state = state;
    if (DS3231_ACTIVE_TIME == (active & DS3231_ACTIVE_TIME))
    	new_state &= ~(1 << 7);
    else
        new_state |= 1 << 7;
    if (DS3231_ACTIVE_SQUARE == (active & DS3231_ACTIVE_SQUARE))
    	new_state &= ~(1 << 6);
    else
        new_state |= 1 << 6;
    if (state != new_state) {
		uint8_t buf[] = {DS3231_CONTROL_REG, state};
		i2c_write_blocking(rtc->i2c_port, rtc->i2c_addr, buf, 2, false);
    }
}

ds3231_sqw_mode_t ds3231_get_sqw_mode(ds3231_rtc_t *rtc) {
    uint8_t reg = DS3231_CONTROL_REG;
    uint8_t state;
    i2c_write_blocking(rtc->i2c_port, rtc->i2c_addr, &reg, 1, true);
    i2c_read_blocking(rtc->i2c_port, rtc->i2c_addr, &state, 1, true);
    return 0 < (state & 0x2) ? DS3231_SQW_MODE_ALARM : DS3231_SQW_MODE_SIGNAL;
}

void ds3231_set_sqw_mode(ds3231_rtc_t *rtc, ds3231_sqw_mode_t mode) {
    uint8_t reg = DS3231_CONTROL_REG;
    uint8_t state;
    i2c_write_blocking(rtc->i2c_port, rtc->i2c_addr, &reg, 1, true);
    i2c_read_blocking(rtc->i2c_port, rtc->i2c_addr, &state, 1, true);
    uint8_t new_state = state;
    switch (mode) {
        case DS3231_SQW_MODE_ALARM:
            new_state |= 0x4;
            new_state &= ~0x18;
            break;
        case DS3231_SQW_MODE_SIGNAL:
            new_state &= ~0x4;
            new_state |= 0x18;
            break;
    }
    if (new_state != state) {
        uint8_t buf[] = {DS3231_CONTROL_REG, new_state};
        i2c_write_blocking(rtc->i2c_port, rtc->i2c_addr, buf, 2, false);
    }
}

void ds3231_set_datetime(ds3231_rtc_t *rtc, ds3231_datetime_t *dt) {
    uint8_t buffer[8];
    buffer[0] = DS3231_DATETIME_REG;

    buffer[1] = DEC2BCD(dt->seconds);
    buffer[2] = DEC2BCD(dt->minutes);
    buffer[3] = DEC2BCD(dt->hour);
    buffer[4] = dt->dotw & 0x07;
    buffer[5] = DEC2BCD(dt->day);
    buffer[6] = DEC2BCD(dt->month);
    buffer[7] = DEC2BCD(dt->year - 2000);

    i2c_write_blocking(rtc->i2c_port, rtc->i2c_addr, buffer, 8, false);

    // After updating the date and time, set the oscillator stop flag
    // (OSF) to 0. Else, it remains at logic 1.
    uint8_t reg = DS3231_STATUS_REG;
    uint8_t status;
    // Read the status register.
    i2c_write_blocking(rtc->i2c_port, rtc->i2c_addr, &reg, 1, true);
    i2c_read_blocking(rtc->i2c_port, rtc->i2c_addr, &status, 1, true);
    // Unset status bit 7.
    const uint8_t new_status = status & (~STATUS_OSF);
    if (new_status != status) {
		uint8_t buf[] = {DS3231_STATUS_REG, new_status};
		i2c_write_blocking(rtc->i2c_port, rtc->i2c_addr, buf, 2, false);
    }
}

void ds3231_get_datetime(ds3231_rtc_t *rtc, ds3231_datetime_t *dt) {
    uint8_t buffer[7];
    uint8_t low, high, val;
    uint8_t reg = DS3231_DATETIME_REG;
    // Go to the start of the date/time register, then read 7 bytes
    i2c_write_blocking(rtc->i2c_port, rtc->i2c_addr, &reg, 1,
                       true);  // `true` to keep control
    i2c_read_blocking(rtc->i2c_port, rtc->i2c_addr, buffer, 7,
                      false);
    
    /* Seconds */
    val = buffer[0];
    dt->seconds = ((val >> 4) * 10) + (val & 0x0f);

    /* Minutes */
    val = buffer[1];
    dt->minutes = ((val >> 4) * 10) + (val & 0x0f);

    /* Hours (1~12 or 0~23) */
    val = buffer[2];
    // Bit 6 is 12-h (1) or 24-h (0) format
    bool mode_is_12_hour = (val >> 6) & 0x1;
    if (mode_is_12_hour) {  // 12-h mode
        // Bit 5 is AM (0) or PM (1)
        // is_pm = (val & 0x20) >> 5;
        // Bit 4 is 10 hour
        high = ((val & 0x10) >> 4) * 10;
        // Bits 3-0 are hour
        low = val & 0x0f;
        dt->hour = high + low;
    } else {  // 24-h mode
        // Bits 5-4 are 20/10 hour
        high = ((val & 0x30) >> 4) * 10;
        // Bits 3-0 are hour
        low = val & 0x0f;
        dt->hour = high + low;
   }

    /* Day of the week (1~7) */
    val = buffer[3];
    // Bits 2-0 are day of the week
    dt->dotw = val & 0x07;
    
    /* Date (day 1~31) */
    val = buffer[4];
    // Bits 5-4 are 10 date
    high = ((val >> 4) * 10);
    // Bits 3-0 are date
    low = val & 0x0f;
    dt->day = high + low;

    /* Month (1-12) */
    val = buffer[5];
    // Bit 4 is 10 month 
    high = ((val & 0x10) >> 4) * 10;
    // Bits 3-0 are month
    low = val & 0x0f;
    dt->month = high + low;
    
    /* Century (0~1) */
    // Bit 7 in same value as month
    // dt->century = (val & 0x80) >> 7;
    
    /* Year (0~99) */
    val = buffer[6];
    // Bits 7-4 are 10 year
    high = ((val & 0xf0) >> 4) * 10;
    // Bits 3-0 are year
    low = val & 0x0f;
    dt->year = 2000 + high + low;
}

void ds3231_isoformat(char *buf, uint8_t buf_size,
                      const ds3231_datetime_t *dt) {
    snprintf(buf, buf_size, "%u-%02u-%02uT%02u:%02u:%02u",
            dt->year, dt->month, dt->day, dt->hour, dt->minutes, dt->seconds);
}

void ds3231_str_date(char *buf, uint8_t buf_size,
                     const ds3231_datetime_t *dt) {
    snprintf(buf, buf_size, "%u-%02u-%02u", dt->year, dt->month,
             dt->day);
}

void ds3231_str_time(char *buf, uint8_t buf_size,
                     const ds3231_datetime_t *dt) {
    snprintf(buf, buf_size, "%02u:%02u:%02u", dt->hour, dt->minutes,
             dt->seconds);
}

void ds3231_ctime(char *buf, uint8_t buf_size,
                  const ds3231_datetime_t *dt) {
    // Day of the week and months start from 1 in the dt structure. We
    // need to subtract 1 from these values to match the correct
    // indices.
    snprintf(buf, buf_size, "%s %s %02u %02u:%02u:%02u %u",
        DS3231_WDAYS[dt->dotw-1], DS3231_MONTHS[dt->month-1], dt->day,
        dt->hour, dt->minutes, dt->seconds, dt->year);
}

bool ds3231_oscillator_is_stopped(ds3231_rtc_t *rtc) {
    uint8_t reg = DS3231_STATUS_REG;
    uint8_t buf;
    // Oscillator Stop Flag: Bit 7 in status register
    i2c_write_blocking(rtc->i2c_port, rtc->i2c_addr, &reg, 1, true);
    i2c_read_blocking(rtc->i2c_port, rtc->i2c_addr, &buf, 1, false);
    return (buf & STATUS_OSF) >> 7;
}

float ds3231_get_temperature(ds3231_rtc_t *rtc) {
    uint8_t buffer[2];
    uint8_t reg = DS3231_TEMPERATURE_REG;
    float frac;
    
    i2c_write_blocking(rtc->i2c_port, rtc->i2c_addr, &reg, 1, true);
    i2c_read_blocking(rtc->i2c_port, rtc->i2c_addr, buffer, 2, false);

    // This quote from the the data sheet (Rev. 10, p.15) explains the
    // lines that follow:
    //
    // Temperature is represented as a 10-bit code with a resolution of
    // 0.25°C and is accessible at location 11h and 12h. The temperature
    // is encoded in two's complement format. The upper 8 bits, the
    // integer portion, are at location 11h and the lower 2 bits, the
    // fractional portion, are in the upper nibble at location 12h.
    
    frac = (float)(buffer[1] >> 6);
    frac *= 0.25;
    
    float result;
    if ((buffer[0] >> 7) & 1) {
        // If bit 7 is set, the number is negative
        result = (float)((~buffer[0]) + 1);
    } else {
        // If bit 7 is not set, the number is positive.
        result = (float)buffer[0];
    }
    result += frac;
    return result;
}

void ds3231_set_alarm(ds3231_rtc_t *rtc, ds3231_alarm_t alarm, const ds3231_alarm_time_t *time, _Bool active) {
    uint8_t second = DEC2BCD(time->second),
            minute = DEC2BCD(time->minute),
            hour = DEC2BCD(time->hour),
            day = DEC2BCD(time->day);
    uint8_t buffer[10] = {0},
            bufIdx = 0;
    switch (alarm) {
        case DS3231_ALARM1: {
            buffer[bufIdx++] = DS3231_ALARM1_REG;
            switch (time->mode.alarm1) {
                case DS3231_AL1_EVERY_SECOND:
                    second |= 0b10000000;
                    minute |= 0b10000000;
                    hour |= 0b10000000;
                    day |= 0b10000000;
                    break;

                case DS3231_AL1_MATCH_SECONDS:
                    second &= 0b01111111;
                    minute |= 0b10000000;
                    hour |= 0b10000000;
                    day |= 0b10000000;
                    break;

                case DS3231_AL1_MATCH_M_S:
                    second &= 0b01111111;
                    minute &= 0b01111111;
                    hour |= 0b10000000;
                    day |= 0b10000000;
                    break;

                case DS3231_AL1_MATCH_H_M_S:
                    second &= 0b01111111;
                    minute &= 0b01111111;
                    hour &= 0b01111111;
                    day |= 0b10000000;
                    break;

                case DS3231_AL1_MATCH_DT_H_M_S:
                    second &= 0b01111111;
                    minute &= 0b01111111;
                    hour &= 0b01111111;
                    day &= 0b01111111;
                    break;

                case DS3231_AL1_MATCH_DY_H_M_S:
                    second &= 0b01111111;
                    minute &= 0b01111111;
                    hour &= 0b01111111;
                    day &= 0b01111111;
                    day |= 0b01000000;
                    break;
            }
            buffer[bufIdx++] = second;
        } break;
        case DS3231_ALARM2: {
            buffer[bufIdx++] = DS3231_ALARM2_REG;
            switch (time->mode.alarm2) {
                case DS3231_AL2_EVERY_MINUTE:
                    minute |= 0b10000000;
                    hour |= 0b10000000;
                    day |= 0b10000000;
                    break;

                case DS3231_AL2_MATCH_M:
                    minute &= 0b01111111;
                    hour |= 0b10000000;
                    day |= 0b10000000;
                    break;

                case DS3231_AL2_MATCH_H_M:
                    minute &= 0b01111111;
                    hour &= 0b01111111;
                    day |= 0b10000000;
                    break;

                case DS3231_AL2_MATCH_DT_H_M:
                    minute &= 0b01111111;
                    hour &= 0b01111111;
                    day &= 0b01111111;
                    break;

                case DS3231_AL2_MATCH_DY_H_M:
                    minute &= 0b01111111;
                    hour &= 0b01111111;
                    day &= 0b01111111;
                    day |= 0b01000000;
                    break;
            }
        } break;
    }
    buffer[bufIdx++] = minute;
    buffer[bufIdx++] = hour;
    buffer[bufIdx++] = day;
    i2c_write_blocking(rtc->i2c_port, rtc->i2c_addr, buffer, bufIdx, false);
    ds3231_clear_alarm(rtc, alarm);
    ds3231_set_alarm_active(rtc, alarm, active);
}

ds3231_alarm_time_t ds3231_get_alarm_time(ds3231_rtc_t *rtc, ds3231_alarm_t alarm) {
    uint8_t reg, length;
    switch (alarm) {
        case DS3231_ALARM1:
            reg = DS3231_ALARM1_REG;
            length = 4;
            break;
        case DS3231_ALARM2:
            reg = DS3231_ALARM1_REG;
            length = 3;
            break;
    }
    i2c_write_blocking(rtc->i2c_port, rtc->i2c_addr, &reg, 1, true);
    uint8_t value[4] = {0};
    i2c_read_blocking(rtc->i2c_port, rtc->i2c_addr, value, length, true);
    ds3231_alarm_time_t result;
    result.day = BCD2DEC(value[0]);
    result.hour = BCD2DEC(value[1]);
    result.minute = BCD2DEC(value[2]);
    uint8_t mode = 0;
    switch (alarm) {
        case DS3231_ALARM1:
            result.second = BCD2DEC(value[3]);
            mode |= ((value[3] & 0b01000000) >> 6);
            mode |= ((value[2] & 0b01000000) >> 5);
            mode |= ((value[1] & 0b01000000) >> 4);
            mode |= ((value[0] & 0b01000000) >> 3);
            mode |= ((value[0] & 0b00100000) >> 1);
            result.mode.alarm1 = (ds3231_alarm1_mode_t)mode;
            break;
        case DS3231_ALARM2:
            result.second = 0;
            mode |= ((value[2] & 0b01000000) >> 5);
            mode |= ((value[1] & 0b01000000) >> 4);
            mode |= ((value[0] & 0b01000000) >> 3);
            mode |= ((value[0] & 0b00100000) >> 1);
            result.mode.alarm2 = (ds3231_alarm2_mode_t)mode;
            break;
    }
    return result;
}

_Bool ds3231_get_alarm_active(ds3231_rtc_t *rtc, ds3231_alarm_t alarm) {
    uint8_t reg = DS3231_CONTROL_REG;
    uint8_t value;
    i2c_write_blocking(rtc->i2c_port, rtc->i2c_addr, &reg, 1, true);
    i2c_read_blocking(rtc->i2c_port, rtc->i2c_addr, &value, 1, true);
    uint8_t bit;
    switch (alarm) {
        case DS3231_ALARM1:
            bit = 0x1;
        break;
        case DS3231_ALARM2:
            bit = 0x2;
        break;
    }
    return 0 < (value & bit);
}

void ds3231_set_alarm_active(ds3231_rtc_t *rtc, ds3231_alarm_t alarm, _Bool state) {
    uint8_t reg = DS3231_CONTROL_REG;
    uint8_t value;
    i2c_write_blocking(rtc->i2c_port, rtc->i2c_addr, &reg, 1, true);
    i2c_read_blocking(rtc->i2c_port, rtc->i2c_addr, &value, 1, true);
    uint8_t bit;
    switch (alarm) {
        case DS3231_ALARM1:
            bit = 0x1;
        break;
        case DS3231_ALARM2:
            bit = 0x2;
        break;
    }
    uint8_t new_value = value;
    if (state)
        new_value |= bit;
    else
        new_value &= ~bit;

    if (new_value != value) {
        uint8_t buf[] = {DS3231_CONTROL_REG, new_value};
        i2c_write_blocking(rtc->i2c_port, rtc->i2c_addr, buf, 2, false);
    }
}

_Bool ds3231_is_alarm_fired(ds3231_rtc_t *rtc, ds3231_alarm_t alarm) {
	uint8_t reg = DS3231_STATUS_REG;
    uint8_t status, bit;
    i2c_write_blocking(rtc->i2c_port, rtc->i2c_addr, &reg, 1, true);
    i2c_read_blocking(rtc->i2c_port, rtc->i2c_addr, &status, 1, true);
    switch (alarm) {
    	case DS3231_ALARM1:
            bit = 0x1;
        break;
        case DS3231_ALARM2:
            bit = 0x2;
        break;
    }
    return 0 < (status & bit);
}

void ds3231_clear_alarm(ds3231_rtc_t *rtc, ds3231_alarm_t alarm) {
	uint8_t reg = DS3231_STATUS_REG;
    uint8_t status, bit;
    i2c_write_blocking(rtc->i2c_port, rtc->i2c_addr, &reg, 1, true);
    i2c_read_blocking(rtc->i2c_port, rtc->i2c_addr, &status, 1, true);
    switch (alarm) {
    	case DS3231_ALARM1:
            bit = 0x1;
        break;
        case DS3231_ALARM2:
            bit = 0x2;
        break;
    }
    const uint8_t new_status = status & (~bit);
    if (new_status != status) {
		uint8_t buf[] = {DS3231_STATUS_REG, new_status};
		i2c_write_blocking(rtc->i2c_port, rtc->i2c_addr, buf, 2, false);
    }
}
