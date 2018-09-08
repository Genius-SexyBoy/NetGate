#include "string.h"
#include "board_nvs.h"

size_t my_nvs_read(nvs_handle handle, const char *name, const char* key, void *value)
{
    esp_err_t err;
     // open
    err = nvs_open(name, NVS_READONLY, &handle);
    if (err != ESP_OK) 
        return err;
    //read
    int32_t max_buffer_size = 4096;
    err = nvs_get_blob(handle, key, value, &max_buffer_size);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) 
        return err;
    // Commit
    err = nvs_commit(handle);
    if (err != ESP_OK) 
        return err;
    // Close
    nvs_close(handle);
    return required_size;
}


esp_err_t my_nvs_write(nvs_handle handle, const char *name, const char* key, void *value)
{
    esp_err_t err;
    // open
    err = nvs_open(name, NVS_READWRITE, &handle);
    if (err != ESP_OK) 
        return err;
    //write
    err = nvs_set_blob(handle, key, value, strlen(value));
    if (err != ESP_OK) 
        return err;
    // Commit
    err = nvs_commit(handle);
    if (err != ESP_OK) 
        return err;
    // Close
    nvs_close(handle);
    return ESP_OK;
}