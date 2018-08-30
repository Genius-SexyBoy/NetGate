


#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "driver/uart.h"

#include "eth.h"
#include "board_uart.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"


#define SERVER_IP "10.10.14.185"
#define SERVER_PORT 1234
#define RECV_BUF_SIZE 1024
#define SEND_BUF_SIZE 1024


extern EventGroupHandle_t eth_event_group;
QueueHandle_t uart_queue_handle;
QueueHandle_t  tcp_queue_handle; 

const int CONNECTED_BIT = BIT0;

typedef struct {
  uint8_t data[512];
  int32_t len;
} data_buf_t;


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
    ESP_LOGI(TAG, "ESP32 Connected to ETH! Start TCP Server....");

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

//      char* send_buf = (char*)calloc(SEND_BUF_SIZE, 1);
//      char* recv_buf = (char*)calloc(SEND_BUF_SIZE, 1);
//      if(send_buf == NULL || recv_buf == NULL )
//      {
//          ESP_LOGE(TAG, "alloc failed, reset chip...");
//          esp_restart();
//      }
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
          static data_buf_t tcp_recv_struct;
          tcp_recv_struct.len = recv(sockfd, (char *)tcp_recv_struct.data, sizeof(tcp_recv_struct.data), 0);
          if(tcp_recv_struct.len > 0)
          {
            //do more chores
            if(xQueueSend(tcp_queue_handle, &tcp_recv_struct, 10/portTICK_RATE_MS) == pdPASS)
            {
               tcp_recv_struct.len = 0;
               memset(tcp_recv_struct.data, 0, sizeof(tcp_recv_struct.data));
            }
          }        
        }
        if(FD_ISSET(sockfd, &write_set))      // writable
        {
          static data_buf_t tcp_send_struct;
          if(xQueueReceive(uart_queue_handle, &tcp_send_struct, 10/portTICK_RATE_MS) == pdTRUE)
          {
            if(tcp_send_struct.len)
            {
              int send_ret = send(sockfd, (char *)tcp_send_struct.data, tcp_send_struct.len, 0);
              if (send_ret == -1)
              {
                ESP_LOGE(TAG, "send data to tcp server failed");
                break;
              }
              else
              {
                ESP_LOGI(TAG, "send data to tcp server succeeded");
                tcp_send_struct.len = 0;
                memset(tcp_send_struct.data, 0, sizeof(tcp_send_struct.data));    
              }
            }
          }                      
        }
        
        vTaskDelay(30/portTICK_RATE_MS);
      }
      if(sockfd > 0)
      {
        close(sockfd);
        ESP_LOGW(TAG, "close socket , sockfd:%d", sockfd);
      }
//      free(send_buf);
//      send_buf = NULL;
//      free(recv_buf);
//      recv_buf = NULL;
      ESP_LOGW(TAG, "reset tcp client and reconnect to tcp server...");
    } // end
    printf( "A stop nonblock...\n");
    vTaskDelete(NULL);
}

void uart_task(void *pvParameter)
{
  static data_buf_t uart_recv_struct;
  while(1)
  {
    //read data from uart
    uart_recv_struct.len = uart_read_bytes(UART_NUM_1, (uint8_t *)uart_recv_struct.data, 512, 10/portTICK_RATE_MS);
    if(uart_recv_struct.len)
    {
      if(xQueueSend(uart_queue_handle, &uart_recv_struct, 10/portTICK_RATE_MS) == pdPASS)
      {
        ESP_LOGI(TAG, "uart send queue error!\n");
        uart_recv_struct.len = 0;
        memset(uart_recv_struct.data, 0, sizeof(uart_recv_struct.data));
      }
    }

    //write data to uart
    static data_buf_t uart_send_struct;
    if(xQueueReceive(tcp_queue_handle, &uart_send_struct, 10/portTICK_RATE_MS) == pdTRUE)
    {
      if(uart_send_struct.len)
      {
        uart_write_bytes(UART_NUM_1, (const char *)uart_send_struct.data, uart_send_struct.len);
        uart_send_struct.len = 0;
        memset(uart_send_struct.data, 0, sizeof(uart_send_struct.data));
      }
    }

    vTaskDelay(30/portTICK_RATE_MS);
  }
  vTaskDelete(NULL);
}

void app_main() 
{
  esp_err_t err = nvs_flash_init();
  eth_event_group = xEventGroupCreate();
  uart_queue_handle = xQueueCreate(100, sizeof(data_buf_t));
   tcp_queue_handle = xQueueCreate(100, sizeof(data_buf_t));
  if (err == ESP_ERR_NVS_NO_FREE_PAGES)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err); 
  ets_delay_us(100000);
  uart_init();
  eth_install(event_handler, NULL);
  eth_config("ESP32-GateWay", inet_addr("10.10.14.99"),
                              inet_addr("10.10.12.1"), inet_addr("255.255.255.0"), 0, 0);
  xTaskCreatePinnedToCore(&uart_task, "uart_task", 4096, NULL, 6, NULL, 1);
  xTaskCreatePinnedToCore(&tcp_client_task, "tcp_client_task", 4096, NULL, 5, NULL, 1);
}
