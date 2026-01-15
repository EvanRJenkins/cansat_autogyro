#pragma once
#include "driver/i2c.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    i2c_port_t port;      // I2C_NUM_0 or I2C_NUM_1
    uint8_t addr_xg;      // 0x6A or 0x6B (accel/gyro)
    uint8_t addr_m;       // 0x1C or 0x1E (mag)
} lsm9ds1_t;

// Initialize device. If addr_xg or addr_m is 0, the driver will auto-detect.
esp_err_t lsm9ds1_init(lsm9ds1_t *dev);

// Read sensors (converted to human-readable units).
// accel: g, gyro: dps, mag: gauss.
esp_err_t lsm9ds1_read_accel(const lsm9ds1_t *dev, float *gx, float *gy, float *gz);
esp_err_t lsm9ds1_read_gyro (const lsm9ds1_t *dev, float *dx, float *dy, float *dz);
esp_err_t lsm9ds1_read_mag  (const lsm9ds1_t *dev, float *mx, float *my, float *mz);

#ifdef __cplusplus
}
#endif

/* References (ideas/specs; no Arduino HAL copied)
1) STMicroelectronics. LSM9DS1 Datasheet — register map; WHO_AM_I values (XG=0x68, M=0x3D);
   I2C addressing via SDO pins (XG: 0x6A/0x6B, M: 0x1C/0x1E); auto-increment bit in CTRL_REG8.
2) Sensitivities for default ranges used here:
   Accel ±2g = 0.061 mg/LSB; Gyro ±245 dps = 8.75 mdps/LSB; Mag ±4 G = 0.14 mG/LSB.
3) Espressif. ESP-IDF I²C Driver — master init and write-then-read pattern
   (i2c_master_write_to_device / i2c_master_write_read_device).
*/