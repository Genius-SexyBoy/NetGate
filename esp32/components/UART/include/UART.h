#ifndef __UART_H__
#define __UART_H__

#include <stdint.h>

#include "esp_system.h"
#include "esp_err.h"
#include "esp_event_loop.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define EX_UART_NUM UART_NUM_1
#define PATTERN_CHR_NUM    (3)    

void UART_Init(void);

#ifdef __cplusplus
}
#endif

#endif
