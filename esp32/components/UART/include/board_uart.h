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

void uart_init(void);

#ifdef __cplusplus
}
#endif

#endif
