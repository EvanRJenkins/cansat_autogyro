#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include "cJSON.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/i2c.h"
#include "esp_log.h"

#include "cpid.h"
// --- RadioLib Includes ---
// #include <RadioLib.h>
// #include "EspHal.h"

// // --- Sensor Includes ---
#include "lsm9ds1_hal.h"

typedef struct 
{
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float pressure;
  float temp;
  float velocityX, velocityY, velocityZ;
  float altitude;
} Data;


// Mutex handle to protect the data
SemaphoreHandle_t dataMutex = NULL;
Data transmittedData; 
float g_currentSP = 0.0f;
// ===================== LOGGING TAGS =====================

static const char *TAG_MAIN  = "MAIN";
static const char *TAG_BARO  = "BARO";
static const char *TAG_GPS   = "GPS";
static const char *TAG_SCD41 = "SCD41";
static const char *TAG_LORA  = "LORA";
static const char *TAG_IMU   = "IMU";
static const char *TAG_TMP   = "TMP";
static const char *TAG_PID   = "PID";

// I2C Pins (DPS310, SCD41, LSM9DS1)
#define I2C_PORT       I2C_NUM_0
#define SDA_GPIO       8
#define SCL_GPIO       9

// LoRa Pins (SX1276)
#define LORA_SCK       18
#define LORA_MISO      19
#define LORA_MOSI      23
#define LORA_CS        5
#define LORA_RST       14
#define LORA_DIO0      26  // G0
#define LORA_DIO1      33  // G1


// ===================== I2C INIT =====================

static void i2c_init(void) {
    i2c_config_t c = {}; 
    c.mode = I2C_MODE_MASTER;
    c.sda_io_num = SDA_GPIO;
    c.scl_io_num = SCL_GPIO;
    c.sda_pullup_en = GPIO_PULLUP_ENABLE;
    c.scl_pullup_en = GPIO_PULLUP_ENABLE;
    c.master.clk_speed = 400000;
    c.clk_flags = 0;

    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &c));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, c.mode, 0, 0, 0));
}
// ===================== LSM9DS1 TASK =====================

static void lsm9ds1_task(void *arg) {
    ESP_LOGI(TAG_IMU, "Starting LSM9DS1 task...");

    if (lsm9ds1_init(I2C_PORT) != ESP_OK) 
    {
        ESP_LOGE(TAG_IMU, "Failed to initialize LSM9DS1");
        vTaskDelete(NULL);
    }

    while (1) {
        float ax, ay, az;
        float gx, gy, gz;

        if (lsm9ds1_read_accel(&ax, &ay, &az) == ESP_OK) {
            ESP_LOGI(TAG_IMU, "Accel (mg): X=%.2f Y=%.2f Z=%.2f", ax, ay, az);

            if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
            {
                transmittedData.accelX = ax;
                transmittedData.accelY = ay;
                transmittedData.accelZ = az;
                //UNLOCK
                xSemaphoreGive(dataMutex);
            }
        }
        if (lsm9ds1_read_gyro(&gx, &gy, &gz) == ESP_OK) {
            ESP_LOGI(TAG_IMU, "Accel (deg/s): X=%.2f Y=%.2f Z=%.2f", gx, gy, gz);

            if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
            {
                transmittedData.gyroX = gx;
                transmittedData.gyroY = gy;
                transmittedData.gyroZ = gz;
                //UNLOCK
                xSemaphoreGive(dataMutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); 
    }
}

// ===================== PID TASK =====================

static void pid_task(void *arg) {
    ESP_LOGI(TAG_PID, "Starting PID task...");
    /*
    if (lsm9ds1_init(I2C_PORT) != ESP_OK) 
    {
        ESP_LOGE(TAG_IMU, "Failed to initialize LSM9DS1");
        vTaskDelete(NULL);
    }
    */
    // Init PID
    PID_Handle_t MyPID = {transmittedData.accelX}; // Init PID handle
    MyPID.PGain = 20.0f; // Set P Gain
    MyPID.IGain = 2.0f; // Set I Gain
    MyPID.DGain = 4.0f; // Set D Gain

    while (1) {
        MyPID.PV = transmittedData.accelX;
        MyPID.SP = g_currentSP;
        PID_Update(&MyPID); // Calculate weights
        printf("PV: %f  SP: %f  CV: %f\n", 
                MyPID.PV, MyPID.SP, MyPID.CV);
        //PID_Process(&MyPID, 100.0f); // Feedback to PV
        // Every 1000 ticks, toggle
        vTaskDelay(pdMS_TO_TICKS(1000)); 
    }
}
void app_main(void) {
    // ---------------------------------------------------------
    // STEP 1: CRITICAL - Create the Mutex BEFORE anything else
    // ---------------------------------------------------------
    dataMutex = xSemaphoreCreateMutex();

    // Check if it failed (e.g., out of memory)
    if (dataMutex == NULL) {
        ESP_LOGE("MAIN", "CRITICAL ERROR: Could not create Mutex!");
        return; // Stop here, do not create tasks
    }

    // ---------------------------------------------------------
    // STEP 2: Initialize I2C (Shared Bus)
    // ---------------------------------------------------------
    i2c_init();

    ESP_LOGI(TAG_MAIN, "Creating tasks...");
    // // IMU Task
    xTaskCreatePinnedToCore(lsm9ds1_task, "lsm9ds1_task", 4096, NULL, 8, NULL, 1);
    xTaskCreatePinnedToCore(pid_task, "pid_task", 4096, NULL, 9, NULL, 1);
}

