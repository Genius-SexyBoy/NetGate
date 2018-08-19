


#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "eth.h"
#include "board_uart.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"


#define BUF_SIZE (1024)

extern EventGroupHandle_t eth_event_group;

SemaphoreHandle_t xSemaphore = NULL;

const int CONNECTED_BIT = BIT0;

TaskHandle_t Mon_Handle;

char middle_buf[128];

static const char *TAG = "main";

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event -> event_id) 
    {
      case SYSTEM_EVENT_ETH_START:
        ESP_LOGI(TAG,"ETH Started");
        break;
      case SYSTEM_EVENT_ETH_GOT_IP:
        ESP_LOGI(TAG,"ETH_GOT_IP");
        xEventGroupSetBits(eth_event_group, CONNECTED_BIT);
        break;
      case SYSTEM_EVENT_ETH_DISCONNECTED:
        ESP_LOGI(TAG,"ETH Disconnected");
        xEventGroupClearBits(eth_event_group, CONNECTED_BIT);
        break;
      default:
        break;
    }
    return ESP_OK;
}



void uart_ondata(uint8_t *data, uint16_t len)
{

  if((data[0] == 0xFE) && (data[1] == 0x64) && (data[2] == 0xFF))
  {
    printf("Shakehands completed!\n");
    printf("Receive %d data\n",len);
  }
  else
  {
    xSemaphoreGive( xSemaphore );
    printf("Receive %d data\n",len);
  }
}


// print pointer data
// para1: data -> start address of print
// para2: len -> print len
// para3: note -> note for print
void print_debug(const char *data, const int len, const char *note)
{
#define COUNT_BYTE_AND_NEW_LINE 0
#define ALL_BINARY_SHOW 0
    printf("\n********** %s [len:%d] start addr:%p **********\n", note, len, data);
    int i = 0;
    for (i = 0; i < len; ++i)
    {
#if !(ALL_BINARY_SHOW)
        if (data[i] < 33 || data[i] > 126)
        {
            if (i > 0 && (data[i - 1] >= 33 && data[i - 1] <= 126) )
            {
                printf(" ");
            }
            printf("%02x ", data[i]);
        }
        else
        {
            printf("%c", data[i]);
        }
#else
        printf("%02x ", data[i]);
#endif

#if COUNT_BYTE_AND_NEW_LINE
        if ((i + 1) % 32 == 0)
        {
            printf("    | %d Bytes\n", i + 1);
        }
#endif
    }
    printf("\n---------- %s End ----------\n", note);
}


static void tcp_client_task(void *pvParameter)
{
    /* Wait for the callback to set the CONNECTED_BIT in the
       event group.
    */
    ESP_LOGI(TAG, "Wait for ESP32 Connect to ETH!");
    xEventGroupWaitBits(eth_event_group, CONNECTED_BIT,
                        false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "ESP32 Connected to WiFi ! Start TCP Server....");

#define SERVER_IP "10.10.15.36"
#define SERVER_PORT 1234
#define RECV_BUF_SIZE 1024
#define SEND_BUF_SIZE 1024

    while(1)
    {
      vTaskDelay(2000 / portTICK_RATE_MS);
      int  sockfd = 0, iResult = 0;

      struct  sockaddr_in serv_addr;
      fd_set read_set, write_set, error_set;

      sockfd  =  socket(AF_INET, SOCK_STREAM, 0);
      if (sockfd == -1)
      {
          ESP_LOGE(TAG, "create socket failed!");
          continue;
      }

      memset( &serv_addr, 0, sizeof (serv_addr));
      serv_addr.sin_family = AF_INET;
      serv_addr.sin_port = htons(SERVER_PORT);
      serv_addr.sin_addr.s_addr  =  inet_addr(SERVER_IP);

      int conn_ret = connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr));

      if (conn_ret == -1)
      {
          ESP_LOGE(TAG, "connect to server failed! errno=%d", errno);
          close(sockfd);
          continue;
      }
      else
      {
          ESP_LOGI(TAG, "connected to tcp server OK, sockfd:%d...", sockfd);
      }
      // set keep-alive
#if 1
      int keepAlive = 1;
      int keepIdle = 10;
      int keepInterval = 1;
      int keepCount = 5;
      int ret = 0;
      ret  = setsockopt(sockfd, SOL_SOCKET,  SO_KEEPALIVE, &keepAlive, sizeof(keepAlive));
      ret |= setsockopt(sockfd, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(keepIdle));
      ret |= setsockopt(sockfd, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(keepInterval));
      ret |= setsockopt(sockfd, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(keepCount));
      if (ret)
      {
          ESP_LOGE(TAG, "set socket keep-alive option failed...");
          close(sockfd);
          continue;
      }
      ESP_LOGI(TAG, "set socket option ok...");
#endif

      struct timeval timeout;
      timeout.tv_sec = 3;
      timeout.tv_usec = 0;

      char* send_buf = (char*)calloc(SEND_BUF_SIZE, 1);
      char* recv_buf = (char*)calloc(SEND_BUF_SIZE, 1);
      if(send_buf == NULL || recv_buf == NULL )
      {
          ESP_LOGE(TAG, "alloc failed, reset chip...");
          esp_restart();
      }
      while  (1)
      {
        FD_ZERO(&read_set);
        FD_SET(sockfd, &read_set);
        write_set = read_set;
        error_set = read_set;
        iResult = select(sockfd  + 1, &read_set, &write_set, &error_set, &timeout);
        if(iResult == -1)
        {
          ESP_LOGE(TAG, "TCP client select failed");
          break;  // reconnect
        }
        if(iResult == 0)
        {
          ESP_LOGI(TAG, "TCP client A select timeout occurred");
          continue;
        }
        if(FD_ISSET(sockfd, &error_set))      // error happen
        {
          ESP_LOGE(TAG, "select error_happend");
          break;
        }
        if(FD_ISSET(sockfd, &read_set))      // readable
        {
          int recv_ret  =  recv(sockfd, recv_buf, RECV_BUF_SIZE, 0);
          if(recv_ret > 0)
          {
            //do more chores
            print_debug(recv_buf, recv_ret, "receive data");
          }
          else
          {
            ESP_LOGW(TAG, "close tcp transmit, would close socket...");
            break;
          }
        }
        if(FD_ISSET(sockfd, &write_set))      // writable
        {
          if( xSemaphoreTake( xSemaphore, ( TickType_t ) 0 ))
          {
            sprintf(send_buf, "hello");   
            // send client data to tcp server
            print_debug(send_buf, strlen(send_buf), "send data");
            int send_ret = send(sockfd, send_buf, strlen(send_buf), 0);
            if (send_ret == -1)
            {
                ESP_LOGE(TAG, "send data to tcp server failed");
                break;
            }
            else
            {
                ESP_LOGI(TAG, "send data to tcp server succeeded");
            }
          }
        }
        vTaskDelay(100/portTICK_RATE_MS);
      }
      if(sockfd > 0)
      {
        close(sockfd);
        ESP_LOGW(TAG, "close socket , sockfd:%d", sockfd);
      }
      free(send_buf);
      send_buf = NULL;
      free(recv_buf);
      recv_buf = NULL;
      ESP_LOGW(TAG, "reset tcp client and reconnect to tcp server...");
    } // end
    printf( "A stop nonblock...\n");
    vTaskDelete(NULL);
    return;
}


void app_main() 
{
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
   xSemaphore = xSemaphoreCreateBinary();

  ESP_ERROR_CHECK( err );
  ets_delay_us(100000);
  eth_install(event_handler, NULL);
  uart_init(uart_ondata);
  xTaskCreatePinnedToCore(&tcp_client_task, "tcp_client_task", 4096, NULL, 5, NULL,1);
}
