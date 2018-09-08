#ifndef _BOARD_NVS_H_
#define _BOARD_NVS_H_

#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"

#ifdef __cplusplus
extern "C" {
#endif

size_t my_nvs_read(nvs_handle handle, const char *name, const char* key, void *value);
esp_err_t my_nvs_write(nvs_handle handle, const char *name, const char* key, void *value);

#ifdef __cplusplus
}
#endif

#endif