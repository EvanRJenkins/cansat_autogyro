#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "cpid.h"
void app_main(void) {
    gpio_reset_pin(GPIO_NUM_3);
    gpio_set_direction(GPIO_NUM_3, GPIO_MODE_OUTPUT);
    int level = 0; 
    PID_Handle_t MyPID = {0.0f}; // Init PID handle
    MyPID.SP = 20.0f; // Set SP
    MyPID.PGain = 20.0f; // Set P Gain
    MyPID.IGain = 2.0f; // Set I Gain
    MyPID.DGain = 4.0f; // Set D Gain

    while (1) {
        gpio_set_level(GPIO_NUM_3, level);
        if (level == 0) {
            level++;
        }
        else level--;
        PID_Update(&MyPID); // Calculate weights
        printf("PV: %f  SP: %f  PGain: %f  IGain: %f  DGain: %f \n", 
                MyPID.PV, MyPID.SP, MyPID.PGain, MyPID.IGain, MyPID.DGain);
        PID_Process(&MyPID, 100.0f); // Feedback to PV
        // Every 1000 ticks, toggle
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

