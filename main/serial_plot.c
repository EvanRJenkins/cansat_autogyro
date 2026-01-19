#include "serial_plot.h"

static const char *TAG = "PLOTTER";

/**
 * @brief Initialize UART for the serial plotter.
 * Call this once in app_main().
 */
void serial_plotter_init(void) {
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = PLOTTER_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_LOGI(TAG, "Initializing UART for Plotter...");

    // Install UART driver, and get the queue.
    ESP_ERROR_CHECK(uart_driver_install(PLOTTER_UART_PORT, PLOTTER_BUF_SIZE, 0, 0, NULL, 0));
    
    // Set UART parameters
    ESP_ERROR_CHECK(uart_param_config(PLOTTER_UART_PORT, &uart_config));
    
    // Set UART pins (using defaults often works for the main USB port)
    ESP_ERROR_CHECK(uart_set_pin(PLOTTER_UART_PORT, PLOTTER_TX_PIN, PLOTTER_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    ESP_LOGI(TAG, "UART Initialized.");
}

/**
 * @brief Send a float value to the plotter.
 * This formats the float into a string followed by a newline '\n'
 * which allows the Python script to use readline().
 * * @param value The sensor reading to plot.
 */
void serial_plotter_send(float val1, float val2, float val3) {
    char data_str[32];
    
    // Format: "123.45\n"
    // %.2f limits it to 2 decimal places to save bandwidth, increase if needed.
    int len = snprintf(data_str, sizeof(data_str), "%.2f, %.2f, %.2f\n", val1, val2, val3);
    
    if (len > 0) {
        uart_write_bytes(PLOTTER_UART_PORT, (const char *)data_str, len);
    }
}
