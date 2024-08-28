#pragma once

#include <stddef.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_system.h"

class Pcf8563 {
 public:
  struct Alarm {
    uint8_t minute;
    uint8_t hour;
    uint8_t day;
    uint8_t weekday;
  };

  struct DateTime {
    uint8_t second;
    uint8_t minute;
    uint8_t hour;
    uint8_t day;
    uint8_t weekday;
    uint8_t month;
    uint16_t year;
  };

  enum InterruptFlags { kAlarmActive = 1 << 0, kTimerActive = (1 << 1) };
  enum ClkOutFreq : uint8_t {
    kClkOutFreq32k768 = 0,
    kClkOutFreq1k024 = 1,
    kClkOutFreq32 = 2,
    kClkOutFreq1 = 3,
  };
  enum TimerFreq : uint8_t {
    kTimerFreq4k096 = 0,
    kTimerFreq64 = 1,
    kTimerFreq1 = 2,
    kTimerFreqp60 = 3,
  };

  // Sets up the device, must be called first.
  esp_err_t Setup(i2c_master_bus_handle_t bus, bool with_outputs);

  // Returns a bitwise OR of InterruptFlags.
  esp_err_t GetAndClearFlags(int *flags);

  // Enables the CLK output.
  esp_err_t SetClockOut(ClkOutFreq mode);

  // Sets the countdown timer.
  esp_err_t SetTimer(TimerFreq mode, uint8_t count);

  // Returns the current value of the countdown timer.
  esp_err_t GetTimer(int *count);

  // Sets an alarm that expires at a particular date.
  esp_err_t SetAlarm(Alarm *alarm, bool match_mins, bool match_hour,
                     bool match_day, bool match_weekday);
  // Gets the current set alarm.
  esp_err_t GetAlarm(Alarm *alarm);

  // Sets the date.
  esp_err_t SetDateTime(DateTime *dateTime);

  // Gets the set data.
  esp_err_t GetDateTime(DateTime *dateTime);

 private:
  template <size_t size>
  esp_err_t Write(uint8_t addr, const uint8_t *data);

  esp_err_t Read(uint8_t addr, uint8_t *data, size_t size);

  i2c_master_dev_handle_t dev_handle_ = nullptr;
};