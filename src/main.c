#include "./sensors/TMP117.h"
#include "./sensors/LSM9DS1.h"
#include "esp_log.h"
#include "driver/i2c.h"


void app_main(void) {
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_ERROR_CHECK(tmp117_init());

    lsm9ds1_t imu = {
        .port = 0,
        .addr_xg = 0,  // 0 => auto-detect (0x6B or 0x6A)
        .addr_m  = 0   // 0 => auto-detect (0x1E or 0x1C)
    };
    if (lsm9ds1_init(&imu) != ESP_OK) {
        printf("LSM9DS1 init failed\n");
        return;
    }
    printf("LSM9DS1 ready (XG=0x%02X, M=0x%02X)\n", imu.addr_xg, imu.addr_m);
    while (1) {
        float t;
        tmp117_read_celsius(&t);
                float ax, ay, az, gx, gy, gz, mx, my, mz;
        if (lsm9ds1_read_accel(&imu, &ax,&ay,&az) == ESP_OK &&
            lsm9ds1_read_gyro (&imu, &gx,&gy,&gz) == ESP_OK &&
            lsm9ds1_read_mag  (&imu, &mx,&my,&mz) == ESP_OK) {

            printf("ACC[g]  %+0.3f %+0.3f %+0.3f | "
                   "GYR[dps] %+0.2f %+0.2f %+0.2f | "
                   "MAG[G]  %+0.3f %+0.3f %+0.3f\n",
                   ax, ay, az, gx, gy, gz, mx, my, mz);
        } else {
            printf("IMU read failed\n");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
