#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"

#include "board_uart.h"


#define BUF_SIZE (1024)

void uart_init(void)
{
//RS232 uart config   
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, GPIO_NUM_16, GPIO_NUM_17,
                                             UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0));

//debug uart config 
    // uart_config.baud_rate = 115200;
    // uart_config.data_bits = UART_DATA_8_BITS;
    // uart_config.parity = UART_PARITY_DISABLE;
    // uart_config.stop_bits = UART_STOP_BITS_1;
    // uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    // ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    // ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
    //                                          UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    // ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0));
}

