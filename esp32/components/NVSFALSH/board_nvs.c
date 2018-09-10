#include "string.h"
#include "board_nvs.h"

size_t my_nvs_read(nvs_handle handle, const char* name, const char* key, char* value)
{
    esp_err_t err;
    size_t size;
     // open
    err = nvs_open(name, NVS_READWRITE, &handle);
    if (err != ESP_OK) 
        ESP_LOGW("nvs", "READ:flash open error!\n");
    //read
    nvs_get_str(handle, key, NULL, &size);
    value = malloc(size);
    nvs_get_str(handle, "key_one", value, &size);
    // Close
    nvs_close(handle);
    return size;
}


void my_nvs_write(nvs_handle handle, const char* name, const char* key, const char* value)
{
    esp_err_t err;
    // open
    err = nvs_open(name, NVS_READWRITE, &handle);
    if (err != ESP_OK) 
        ESP_LOGW("nvs", "WRITE:flash open error!\n");
    //write
    err = nvs_set_str(handle, key, value);
    if (err != ESP_OK) 
        ESP_LOGW("nvs", "WRITE:flash set error!\n");
    // Commit
    err = nvs_commit(handle);
    if (err != ESP_OK) 
        ESP_LOGW("nvs", "WRITE:flash commit error!\n");
    // Close
    nvs_close(handle);
}