#include "driver/i2c.h"
#include "esp_log.h"
#include <stdint.h>

#define I2C_PORT        I2C_NUM_0

#define TMP117_ADDR     0x48        // default address per datasheet
// TMP117 register map (datasheet)
#define TMP117_REG_TEMP     0x00
#define TMP117_REG_CONFIG   0x01
#define TMP117_REG_T_HIGH   0x02
#define TMP117_REG_T_LOW    0x03
#define TMP117_REG_EEPROMUL 0x04
#define TMP117_REG_EEPROM1  0x05
#define TMP117_REG_EEPROM2  0x06
#define TMP117_REG_TEMP_OFST 0x07
#define TMP117_REG_EEPROM3  0x08
#define TMP117_REG_DEVICE_ID 0x0F    // returns 0x0117

const char *TAG = "TMP117";

esp_err_t tmp117_write_u16(uint8_t reg, uint16_t value) {
    uint8_t buf[3] = { reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };
    return i2c_master_write_to_device(I2C_PORT, TMP117_ADDR, buf, sizeof(buf), pdMS_TO_TICKS(100));
}

esp_err_t tmp117_read_u16(uint8_t reg, uint16_t *out) {
    uint8_t rx[2];
    esp_err_t err = i2c_master_write_read_device(
        I2C_PORT, TMP117_ADDR, &reg, 1, rx, 2, pdMS_TO_TICKS(100));
    if (err != ESP_OK) return err;
    *out = ((uint16_t)rx[0] << 8) | rx[1];
    return ESP_OK;
}

// Optional: put TMP117 in continuous conversion, 1 Hz, no averaging (see datasheet CONFIG bits)
esp_err_t tmp117_init(void) {
    // CONFIG defaults are usually fine; here’s a safe example:
    // bitfields (refer to datasheet): MODE=continuous(00), CONV=1Hz (000), AVG=1 (00)
    // leave rest default; write only if you need to.
    // For illustration, we just read device ID to validate presence.
    uint16_t whoami = 0;
    ESP_ERROR_CHECK(tmp117_read_u16(TMP117_REG_DEVICE_ID, &whoami));
    if (whoami != 0x0117) {
        ESP_LOGW(TAG, "Unexpected WHOAMI: 0x%04X", whoami);
    }
    return ESP_OK;
}

esp_err_t tmp117_read_celsius(float *degC) {
    uint16_t raw16 = 0;
    esp_err_t err = tmp117_read_u16(TMP117_REG_TEMP, &raw16);
    if (err != ESP_OK) return err;          // don't convert on failure

    int16_t raw = (int16_t)raw16;           // two's complement
    *degC = (float)raw / 128.0f;            // 0.0078125 °C/LSB

    ESP_LOGI(TAG, "Temperature: %.3f C", (double)*degC);
    // or: printf("Temperature: %.3f C\n", (double)*degC);

    return ESP_OK;
}
