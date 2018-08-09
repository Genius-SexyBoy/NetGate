


#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_wifi.h"

#include "eth.h"
#include "board_uart.h"

#include "mongoose.h"

#define EXAMPLE_ESP_WIFI_MODE_AP   CONFIG_ESP_WIFI_MODE_AP //TRUE:AP FALSE:STA
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_MAX_STA_CONN       CONFIG_MAX_STA_CONN

#define BUF_SIZE (1024)

TaskHandle_t th[12];

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
        //xTaskCreatePinnedToCore(mongoose_task, "mongoose_task", 4096, NULL, 2, &th[0], 1);  //creat tcp task
        break;
      case SYSTEM_EVENT_ETH_DISCONNECTED:
        ESP_LOGI(TAG,"ETH Disconnected");
        //vTaskDelete(th[0]);               //delete the task
        break;
      case SYSTEM_EVENT_ETH_STOP:
        ESP_LOGI(TAG,"ETH Stopped");
        break;
      default:
        break;
  }
    return ESP_OK;
}

void wifi_init_sta()
{
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL) );

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");
    ESP_LOGI(TAG, "connect to ap SSID:%s password:%s",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
}

static void ev_handler(struct mg_connection *nc, int ev, void *ev_data) {
  struct mbuf *io = &nc->recv_mbuf;

  switch (ev) 
  {
    case MG_EV_RECV:
      // This event handler implements simple TCP echo server
//      mg_send(nc, io->buf, io->len);  // Echo received data back
         ESP_LOGI(TAG,"%s",io->buf);
         mbuf_remove(io, io->len);      // Discard data from recv buffer
         break;
    default:break;
  }
}

static void mongoose_task(void *pvParameter)
{
   struct mg_mgr mgr;
  mg_mgr_init(&mgr, NULL);
  mg_bind(&mgr, "1234", ev_handler);
  while(1) 
  {  
    mg_mgr_poll(&mgr, 1000);
    vTaskDelay(1);
  }  
}

void uart_ondata(uint8_t *data, uint16_t len)
{
  uint8_t *pa = NULL;
  uint8_t *pb = NULL;
  uint8_t patter[20] = {0x0A, 0x0B, 0x0C, 0};

  pa = (uint8_t *)strstr((const char *)data,(const char *)patter);

  if(pa) 
  {
    vTaskDelay(500 / portTICK_RATE_MS);
    printf("---------------------test-----------------\n");
  } 
  else 
  {
    printf("SB\n");
  }
}




void app_main() 
{
  uart_init(uart_ondata);
  ESP_LOGI(TAG, "NetGate\n");
  tcpip_adapter_init();
  eth_install(event_handler, NULL);
  
}