#ifndef _BOARD_UART_H_
#define _BOARD_UART_H_

#include <stdint.h>

#include "esp_system.h"
#include "esp_err.h"
#include "esp_event_loop.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// #define EX_UART_NUM UART_NUM_1
// #define PATTERN_CHR_NUM    (3)  

typedef void (*callback_t)(uint8_t *data, uint16_t len);

void uart_init(callback_t cb);

#ifdef __cplusplus
}
#endif

#endif
