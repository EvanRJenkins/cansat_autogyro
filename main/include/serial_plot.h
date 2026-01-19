#ifndef SERIAL_PLOT_H
#define SERIAL_PLOT_H

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

// --- CONFIGURATION ---
// UART_NUM_0 is usually the default USB-UART bridge on dev boards.
// If using the native USB port (USB Serial/JTAG), this setup might differ.
#define PLOTTER_UART_PORT      UART_NUM_0
#define PLOTTER_BAUD_RATE      230400
#define PLOTTER_TX_PIN         UART_PIN_NO_CHANGE // Uses default pins for the board
#define PLOTTER_RX_PIN         UART_PIN_NO_CHANGE
#define PLOTTER_BUF_SIZE       1024

/**
 * @brief Initialize UART for the serial plotter.
 * Call this once in app_main().
 */
void serial_plotter_init(void);


/**
 * @brief Send a float value to the plotter.
 * This formats the float into a string followed by a newline '\n'
 * which allows the Python script to use readline().
 * * @param value The sensor reading to plot.
 */
void serial_plotter_send(float val1, float val2, float val3);
#endif // Define header
