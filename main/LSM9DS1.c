#include "lsm9ds1.h"
#include "driver/i2c.h"
#include "esp_check.h"

#define WHO_AM_I_XG   0x0F
#define WHO_AM_I_M    0x0F
#define WHO_XG_VAL    0x68
#define WHO_M_VAL     0x3D

// XG registers
#define CTRL_REG1_G   0x10
#define CTRL_REG6_XL  0x20
#define CTRL_REG8     0x22
#define OUT_X_L_G     0x18
#define OUT_X_L_XL    0x28

// M registers
#define CTRL_REG1_M   0x20
#define CTRL_REG2_M   0x21
#define CTRL_REG3_M   0x22
#define CTRL_REG4_M   0x23
#define CTRL_REG5_M   0x24
#define OUT_X_L_M     0x28

static esp_err_t wr8(i2c_port_t p, uint8_t a, uint8_t r, uint8_t v) {
    uint8_t b[2] = {r, v};
    return i2c_master_write_to_device(p, a, b, 2, pdMS_TO_TICKS(50));
}
static esp_err_t rd(i2c_port_t p, uint8_t a, uint8_t r, uint8_t *buf, size_t n) {
    return i2c_master_write_read_device(p, a, &r, 1, buf, n, pdMS_TO_TICKS(50));
}
static inline int16_t be2i16(uint8_t lo, uint8_t hi) { return (int16_t)((hi<<8)|lo); }

static bool detect(i2c_port_t p, uint8_t *addr_try, uint8_t who_reg, uint8_t who_val, uint8_t *out) {
    for (int i=0;i<2;i++) {
        uint8_t v;
        if (rd(p, addr_try[i], who_reg, &v, 1) == ESP_OK && v == who_val) { *out = addr_try[i]; return true; }
    }
    return false;
}

esp_err_t lsm9ds1_init(lsm9ds1_t *dev) {
    if (!dev) return ESP_ERR_INVALID_ARG;

    // Auto-detect addresses if not provided
    if (dev->addr_xg == 0) {
        uint8_t xg_list[2] = {0x6B, 0x6A};
        if (!detect(dev->port, xg_list, WHO_AM_I_XG, WHO_XG_VAL, &dev->addr_xg)) return ESP_FAIL;
    }
    if (dev->addr_m == 0) {
        uint8_t m_list[2] = {0x1E, 0x1C};
        if (!detect(dev->port, m_list, WHO_AM_I_M, WHO_M_VAL, &dev->addr_m)) return ESP_FAIL;
    }

    // Enable auto-increment & BDU on XG
    ESP_RETURN_ON_ERROR(wr8(dev->port, dev->addr_xg, CTRL_REG8, 0x44), "lsm9ds1", "CTRL_REG8");

    // Gyro: ODR=119 Hz (011), FS=245 dps (00)
    ESP_RETURN_ON_ERROR(wr8(dev->port, dev->addr_xg, CTRL_REG1_G, 0x60), "lsm9ds1", "CTRL_REG1_G");

    // Accel: ODR=119 Hz (011), FS=±2 g (00)
    ESP_RETURN_ON_ERROR(wr8(dev->port, dev->addr_xg, CTRL_REG6_XL, 0x60), "lsm9ds1", "CTRL_REG6_XL");

    // Mag: BDU=1
    ESP_RETURN_ON_ERROR(wr8(dev->port, dev->addr_m, CTRL_REG5_M, 0x40), "lsm9ds1", "CTRL_REG5_M");
    // TEMP_COMP=1, ODR=20 Hz (010)
    ESP_RETURN_ON_ERROR(wr8(dev->port, dev->addr_m, CTRL_REG1_M, 0x80 | (0x02<<2)), "lsm9ds1", "CTRL_REG1_M");
    // FS=±4 G
    ESP_RETURN_ON_ERROR(wr8(dev->port, dev->addr_m, CTRL_REG2_M, 0x00), "lsm9ds1", "CTRL_REG2_M");
    // Continuous-conversion
    ESP_RETURN_ON_ERROR(wr8(dev->port, dev->addr_m, CTRL_REG3_M, 0x00), "lsm9ds1", "CTRL_REG3_M");
    // Default for CTRL_REG4_M
    ESP_RETURN_ON_ERROR(wr8(dev->port, dev->addr_m, CTRL_REG4_M, 0x00), "lsm9ds1", "CTRL_REG4_M");

    return ESP_OK;
}

esp_err_t lsm9ds1_read_gyro(const lsm9ds1_t *dev, float *dx, float *dy, float *dz) {
    uint8_t b[6];
    ESP_RETURN_ON_ERROR(rd(dev->port, dev->addr_xg, OUT_X_L_G, b, 6), "lsm9ds1", "rd gyro");
    const float sens_dps = 0.00875f; // 8.75 mdps/LSB @ ±245 dps
    *dx = be2i16(b[0], b[1]) * sens_dps;
    *dy = be2i16(b[2], b[3]) * sens_dps;
    *dz = be2i16(b[4], b[5]) * sens_dps;
    return ESP_OK;
}

esp_err_t lsm9ds1_read_accel(const lsm9ds1_t *dev, float *gx, float *gy, float *gz) {
    uint8_t b[6];
    ESP_RETURN_ON_ERROR(rd(dev->port, dev->addr_xg, OUT_X_L_XL, b, 6), "lsm9ds1", "rd accel");
    const float sens_g = 0.000061f; // 0.061 mg/LSB -> g
    *gx = be2i16(b[0], b[1]) * sens_g;
    *gy = be2i16(b[2], b[3]) * sens_g;
    *gz = be2i16(b[4], b[5]) * sens_g;
    return ESP_OK;
}

esp_err_t lsm9ds1_read_mag(const lsm9ds1_t *dev, float *mx, float *my, float *mz) {
    uint8_t b[6];
    ESP_RETURN_ON_ERROR(rd(dev->port, dev->addr_m, OUT_X_L_M, b, 6), "lsm9ds1", "rd mag");
    const float sens_gs = 0.00014f; // 0.14 mG/LSB -> gauss
    *mx = be2i16(b[0], b[1]) * sens_gs;
    *my = be2i16(b[2], b[3]) * sens_gs;
    *mz = be2i16(b[4], b[5]) * sens_gs;
    return ESP_OK;
}
