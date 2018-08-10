


#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_wifi.h"

#include "eth.h"
#include "board_uart.h"

#include "mongoose.h"


#define BUF_SIZE (1024)

TaskHandle_t Mon_Handle;
struct mg_mgr mgr;
struct mg_connection *c;

static void mongoose_task(void *pvParameter);

static const char *TAG = "main";

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event -> event_id) {
      case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
      case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "got ip:%s",
                 ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
        break;
      case SYSTEM_EVENT_AP_STACONNECTED:
        ESP_LOGI(TAG, "station:"MACSTR" join, AID=%d",
                 MAC2STR(event->event_info.sta_connected.mac),
                 event->event_info.sta_connected.aid);
        break;
      case SYSTEM_EVENT_AP_STADISCONNECTED:
        ESP_LOGI(TAG, "station:"MACSTR"leave, AID=%d",
                 MAC2STR(event->event_info.sta_disconnected.mac),
                 event->event_info.sta_disconnected.aid);
        break;
      case SYSTEM_EVENT_STA_DISCONNECTED:
        esp_wifi_connect();
        break;
      case SYSTEM_EVENT_ETH_START:
        ESP_LOGI(TAG,"ETH Started");
        break;
      case SYSTEM_EVENT_ETH_CONNECTED:
        ESP_LOGI(TAG,"ETH Connected");
        break;
      case SYSTEM_EVENT_ETH_GOT_IP:
        ESP_LOGI(TAG,"ETH_GOT_IP");
        xTaskCreatePinnedToCore(mongoose_task, "mongoose_task", 4096, NULL, 2, &Mon_Handle, 1);  //creat tcp task
        break;
      case SYSTEM_EVENT_ETH_DISCONNECTED:
        ESP_LOGI(TAG,"ETH Disconnected");
        vTaskDelete(Mon_Handle);               //delete the task
        break;
      case SYSTEM_EVENT_ETH_STOP:
        ESP_LOGI(TAG,"ETH Stopped");
        break;
      default:
        break;
  }
    return ESP_OK;
}


static void ev_handler(struct mg_connection *c, int ev, void *ev_data) 
{
  switch (ev) 
  {
    case MG_EV_CONNECT:
         ESP_LOGI(TAG,"%s","conntine to");
         mg_printf(c, "%s", "hi there");
         break;
    case MG_EV_RECV:
         ESP_LOGI(TAG,"%s","hello,kangkang\n");
    default:break;
  }
}

static void mongoose_task(void *pvParameter)
{
  
  mg_mgr_init(&mgr, NULL);
  c = mg_connect(&mgr, "10.10.14.15:1234", ev_handler);
  while(1) 
  {  
    mg_mgr_poll(&mgr, 1000);
    vTaskDelay(1);
  }  
}

void uart_ondata(uint8_t *data, uint16_t len)
{
//  char str_buf[20];
//  sprintf(str_buf, "%x", *data);

  if((data[0] == 0xFE) && (data[1] == 0x64) && (data[2] == 0xFF))
  {
    printf("Shakehands completed!\n");
    printf("Receive %d data\n",len);
  }
  else
  {
    mg_printf(c, "%s", data);                  //Send to server
    printf("Send to Server!\n");
    printf("Receive %d data\n",len);
  }
}




void app_main() 
{
  ets_delay_us(100000);
  uart_init(uart_ondata);
  ESP_LOGI(TAG, "NetGate\n");
  tcpip_adapter_init();
  eth_install(event_handler, NULL);
}