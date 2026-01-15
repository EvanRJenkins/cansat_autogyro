#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
void app_main(void) {
    gpio_reset_pin(GPIO_NUM_3);
    gpio_set_direction(GPIO_NUM_3, GPIO_MODE_OUTPUT);
    int level = 0;
    while (1) {
        gpio_set_level(GPIO_NUM_3, level);
        if (level == 0) {
            level++;
        }
        else level--;
        // Every 1000 ticks, toggle
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

