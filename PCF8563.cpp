
#include "PCF8563.h"

#include <stdlib.h>
#include <sys/time.h>
#include <time.h>

#include "driver/i2c.h"
#include "esp_system.h"

namespace {
constexpr auto PCF8563_READ_ADDR = 0xA3;
constexpr auto PCF8563_WRITE_ADDR = 0xA2;

constexpr auto BinToBCD(uint8_t bin) { return ((bin / 10) << 4) + (bin % 10); }
}  // namespace

esp_err_t Pcf8563::Write(uint8_t addr, const uint8_t *data, size_t count) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, PCF8563_WRITE_ADDR, true);
  i2c_master_write_byte(cmd, addr, true);
  i2c_master_write(cmd, data, count, true);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(port_, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

esp_err_t Pcf8563::Read(uint8_t addr, uint8_t *data, size_t count) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, PCF8563_WRITE_ADDR, true);
  i2c_master_write_byte(cmd, addr, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, PCF8563_READ_ADDR, true);
  i2c_master_read(cmd, data, count, I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(port_, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);

  return ret;
}

esp_err_t Pcf8563::Setup(bool with_outputs) {
  i2c_param_config(port_, &i2c_config_);
  esp_err_t ret = i2c_driver_install(port_, i2c_config_.mode, 0, 0, 0);
  if (ret != ESP_OK) {
    return ret;
  }
  uint8_t tmp = 0b00000000;
  ret = Write(0x00, &tmp, 1);
  if (ret != ESP_OK) {
    return ret;
  }
  const uint8_t mode = with_outputs ? 0b11 : 0;
  ret = Write(0x01, &mode, 1);
  if (ret != ESP_OK) {
    return ret;
  }

  return ESP_OK;
}

esp_err_t Pcf8563::GetAndClearFlags(int *out) {
  uint8_t flags;

  esp_err_t ret = Read(0x01, &flags, 1);
  if (ret != ESP_OK) {
    return ret;
  }
  const uint8_t cleared = flags & 0b00010011;
  ret = Write(0x01, &cleared, 1);
  if (ret != ESP_OK) {
    return ret;
  }
  *out = flags;
  return ESP_OK;
}

esp_err_t Pcf8563::SetClockOut(ClkOutFreq mode) {
  const uint8_t value = mode | (1 << 7);
  esp_err_t ret = Write(0x0D, &value, 1);
  if (ret != ESP_OK) {
    return ret;
  }
  return ESP_OK;
}

esp_err_t Pcf8563::SetTimer(TimerFreq mode, uint8_t count) {
  const uint8_t value = mode | (1 << 7);
  esp_err_t ret = Write(0x0E, &value, 1);
  if (ret != ESP_OK) {
    return ret;
  }
  ret = Write(0x0F, &count, 1);
  if (ret != ESP_OK) {
    return ret;
  }
  return ESP_OK;
}

esp_err_t Pcf8563::GetTimer(int *count) {
  uint8_t val;

  esp_err_t ret = Read(0x0F, &val, 1);
  if (ret != ESP_OK) {
    return ret;
  }
  *count = val;
  return ESP_OK;
}

esp_err_t Pcf8563::SetAlarm(Pcf8563::Alarm *alarm, bool match_mins,
                            bool match_hour, bool match_day,
                            bool match_weekday) {
  if ((alarm->minute >= 60 && alarm->minute != 80) ||
      (alarm->hour >= 24 && alarm->hour != 80) ||
      (alarm->day > 32 && alarm->day != 80) ||
      (alarm->weekday > 6 && alarm->weekday != 80)) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t buffer[4];

  buffer[0] = BinToBCD(alarm->minute) | (match_mins ? 0x80 : 0);
  buffer[1] = BinToBCD(alarm->hour) | (match_hour ? 0x80 : 0);
  buffer[2] = BinToBCD(alarm->day) | (match_day ? 0x80 : 0);
  buffer[3] = BinToBCD(alarm->weekday) | (match_weekday ? 0x80 : 0);

  esp_err_t ret = Write(0x09, buffer, sizeof(buffer));
  if (ret != ESP_OK) {
    return ret;
  }

  return ESP_OK;
}

esp_err_t Pcf8563::GetAlarm(Pcf8563::Alarm *alarm) {
  uint8_t buffer[4];

  esp_err_t ret = Read(0x09, buffer, sizeof(buffer));
  if (ret != ESP_OK) {
    return ret;
  }

  alarm->minute = (((buffer[0] >> 4) & 0x0F) * 10) + (buffer[0] & 0x0F);
  alarm->hour = (((buffer[1] >> 4) & 0x0B) * 10) + (buffer[1] & 0x0F);
  alarm->day = (((buffer[2] >> 4) & 0x0B) * 10) + (buffer[2] & 0x0F);
  alarm->weekday = (((buffer[3] >> 4) & 0x08) * 10) + (buffer[3] & 0x07);

  return ESP_OK;
}

esp_err_t Pcf8563::SetDateTime(Pcf8563::DateTime *dateTime) {
  if (dateTime->second >= 60 || dateTime->minute >= 60 ||
      dateTime->hour >= 24 || dateTime->day > 32 || dateTime->weekday > 6 ||
      dateTime->month > 12 || dateTime->year < 1900 || dateTime->year >= 2100) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t buffer[7];

  buffer[0] = BinToBCD(dateTime->second) & 0x7F;
  buffer[1] = BinToBCD(dateTime->minute) & 0x7F;
  buffer[2] = BinToBCD(dateTime->hour) & 0x3F;
  buffer[3] = BinToBCD(dateTime->day) & 0x3F;
  buffer[4] = BinToBCD(dateTime->weekday) & 0x07;
  buffer[5] = BinToBCD(dateTime->month) & 0x1F;

  if (dateTime->year >= 2000) {
    buffer[5] |= 0x80;
    buffer[6] = BinToBCD(dateTime->year - 2000);
  } else {
    buffer[6] = BinToBCD(dateTime->year - 1900);
  }

  esp_err_t ret = Write(0x02, buffer, sizeof(buffer));
  if (ret != ESP_OK) {
    return ret;
  }

  return 0;
}

esp_err_t Pcf8563::GetDateTime(Pcf8563::DateTime *dateTime) {
  uint8_t buffer[7];
  esp_err_t ret;

  ret = Read(0x02, buffer, sizeof(buffer));
  if (ret != ESP_OK) {
    return ret;
  }

  dateTime->second = (((buffer[0] >> 4) & 0x07) * 10) + (buffer[0] & 0x0F);
  dateTime->minute = (((buffer[1] >> 4) & 0x07) * 10) + (buffer[1] & 0x0F);
  dateTime->hour = (((buffer[2] >> 4) & 0x03) * 10) + (buffer[2] & 0x0F);
  dateTime->day = (((buffer[3] >> 4) & 0x03) * 10) + (buffer[3] & 0x0F);
  dateTime->weekday = (buffer[4] & 0x07);
  dateTime->month = ((buffer[5] >> 4) & 0x01) * 10 + (buffer[5] & 0x0F);
  dateTime->year = 1900 + ((buffer[6] >> 4) & 0x0F) * 10 + (buffer[6] & 0x0F);

  if (buffer[5] & 0x80) {
    dateTime->year += 100;
  }

  if (buffer[0] & 0x80)  // Clock integrity not guaranted
  {
    return ESP_ERR_INVALID_RESPONSE;
  }

  return ESP_OK;
}
