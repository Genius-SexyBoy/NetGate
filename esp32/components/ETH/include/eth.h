#ifndef __ETH_H__
#define __ETH_H__

#include <stdint.h>

#include "esp_system.h"
#include "esp_err.h"
#include "esp_event_loop.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t eth_install(system_event_cb_t cb, void *ctx);
esp_err_t eth_config(const char * hostname, uint32_t local_ip, uint32_t gateway, 
                                uint32_t subnet, uint32_t dns1, uint32_t dns2);

#ifdef __cplusplus
}
#endif

#endif