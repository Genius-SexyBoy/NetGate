#ifndef _BOARD_NVS_H_
#define _BOARD_NVS_H_

#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif

size_t my_nvs_read(nvs_handle handle, const char* name, const char* key, char* value);
void my_nvs_write(nvs_handle handle, const char* name, const char* key, const char* value);

#ifdef __cplusplus
}
#endif

#endif