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

// Include Evan's serial plot code
#include "serial_plot.h"
typedef struct 
{
  float ax_mps2, ay_mps2, az_mps2;
  float gyroX, gyroY, gyroZ;
  float pressure;
  float temp;
  float velocityX, velocityY, velocityZ;
  float altitude;
} Data;


// Mutex handle to protect the data
SemaphoreHandle_t dataMutex = NULL;
Data transmittedData; 
// Create variables to hold roll (phi) and pitch (theta) angles
float g_phiHat_deg = 0.0f;
float g_thetaHat_deg = 0.0f;
// Create variable for PID SP
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
static const char *TAG_SERIAL_PLOT   = "SERIAL_PLOT";
static const char *TAG_PR_ESTIMATES   = "PR_ESTIMATES";

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

// Factors for math in pitch and roll estimate task
#define RAD_TO_DEG 57.3f
#define G_MPS2 981.0f

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
    float ax, ay, az;
    float gx, gy, gz;
    while (1) {
        if (lsm9ds1_read_accel(&ax, &ay, &az) == ESP_OK) {
            ESP_LOGI(TAG_IMU, "Accel (mg): X=%.2f Y=%.2f Z=%.2f", ax, ay, az);

            if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
            {
                transmittedData.ax_mps2 = ax;
                transmittedData.ay_mps2 = ay;
                transmittedData.az_mps2 = az;
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
        vTaskDelay(pdMS_TO_TICKS(20)); 
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
    PID_Handle_t MyPID = {transmittedData.ax_mps2}; // Init PID handle
    MyPID.PGain = 20.0f; // Set P Gain
    MyPID.IGain = 2.0f; // Set I Gain
    MyPID.DGain = 4.0f; // Set D Gain

    while (1) {
        MyPID.PV = transmittedData.ax_mps2;
        MyPID.SP = g_currentSP;
        PID_Update(&MyPID); // Calculate weights
        //printf("PV: %f  SP: %f  CV: %f\n", 
        //        MyPID.PV, MyPID.SP, MyPID.CV);
        //PID_Process(&MyPID, 100.0f); // Feedback to PV
        // Every X ticks, toggle
        vTaskDelay(pdMS_TO_TICKS(20)); 
    }
}

// ===================== PITCH & ROLL ESTIMATES TASK =====================

static void pr_estimates_task(void *arg) {
    ESP_LOGI(TAG_PR_ESTIMATES, "Starting Pitch & Roll Estimates task...");
    while (1) {
        // Calculate roll and pitch angle estimates
        g_phiHat_deg = atanf(transmittedData.ay_mps2 / transmittedData.az_mps2) * RAD_TO_DEG;
        g_thetaHat_deg = asinf(transmittedData.ax_mps2 / G_MPS2) * RAD_TO_DEG;
        // Delay
        vTaskDelay(pdMS_TO_TICKS(20)); 
    }
}

// ===================== SERIAL PLOT TASK =====================

static void serial_plot_task(void *arg) {
    ESP_LOGI(TAG_SERIAL_PLOT, "Starting serial plot task...");

    // Init serial plotter
    serial_plotter_init();

    while (1) {
        // Send new x acceleration sample
        serial_plotter_send(g_phiHat_deg, g_thetaHat_deg);
        vTaskDelay(pdMS_TO_TICKS(20)); 
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
    xTaskCreatePinnedToCore(lsm9ds1_task, "lsm9ds1_task", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(pr_estimates_task, "pr_estimates_task", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(pid_task, "pid_task", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(serial_plot_task, "serial_plot_task", 4096, NULL, 4, NULL, 1);
}

