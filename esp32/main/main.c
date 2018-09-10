


#include <stdio.h>
#include <string.h>
#include <stdlib.h> 

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
#include "board_nvs.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"


nvs_handle my_handle;


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




static void tcp_client_task(void *pvParameter)
{
  /* Wait for the callback to set the CONNECTED_BIT in the
      event group.
  */
  ESP_LOGI(TAG, "Wait for ESP32 Connect to ETH!");
  xEventGroupWaitBits(eth_event_group, CONNECTED_BIT,
                      false, true, portMAX_DELAY);
  ESP_LOGI(TAG, "ESP32 Connected to ETH!Ready start TCP Server....");

  /* 
  *  Read the tcp server ip_addr 
  *  and port from the nvs flash
  */
  size_t size;
  esp_err_t err = nvs_open("Remote", NVS_READWRITE, &my_handle);
  if (err != ESP_OK) 
    ESP_LOGW(TAG, "TCP:flash open error!\n");
  nvs_get_str(my_handle, "IP", NULL, &size);
  char ip_addr[size];
  nvs_get_str(my_handle, "IP", ip_addr, &size);

  nvs_get_str(my_handle, "Port", NULL, &size);
  char port[10];
  nvs_get_str(my_handle, "Port", port, &size);
  int ip_port = atoi(port);
  nvs_close(my_handle);

  
  printf("%d\n", ip_port);
  while(1)
  {
    vTaskDelay(1000 / portTICK_RATE_MS);
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
    serv_addr.sin_port = htons(ip_port);
    serv_addr.sin_addr.s_addr  =  inet_addr(ip_addr);

    int conn_ret = connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr));

    if (conn_ret == -1)
    {
      //ESP_LOGE(TAG, "%s", serv_addr.sin_addr.s_addr);
      ESP_LOGE(TAG, "%d", serv_addr.sin_port);
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
    while  (1)
    {
      FD_ZERO(&read_set);
      FD_SET(sockfd, &read_set);
      write_set = read_set;
      error_set = read_set;
      iResult = select(sockfd + 1, &read_set, &write_set, &error_set, &timeout);
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

void at_commend_task(void *pvParameter)
{
  static data_buf_t at_commend_struct;
  const char ch = '=';
  char* ret = NULL;
  while(1)
  {
    at_commend_struct.len = uart_read_bytes(UART_NUM_0, (uint8_t *)at_commend_struct.data, 512, 10/portTICK_RATE_MS);
    if(at_commend_struct.len)
    {
      if((at_commend_struct.data[0] == 'A') && (at_commend_struct.data[1] == 'T') && (at_commend_struct.data[2] == '+'))
      {
        if((at_commend_struct.data[3] == 'L') && (at_commend_struct.data[4] == 'C'))
        {
          if((at_commend_struct.data[5] == 'I') && (at_commend_struct.data[6] == 'P') && (at_commend_struct.data[7] == '='))
          {
            //AT+LCIP=
            
          }
          else if((at_commend_struct.data[5] == 'G') && (at_commend_struct.data[6] == 'W') && (at_commend_struct.data[7] == '='))
          {
            ESP_LOGI(TAG, "Set local gateway success!\n");
          }
        }
        else if((at_commend_struct.data[3] == 'R') && (at_commend_struct.data[4] == 'M'))
        {
          if((at_commend_struct.data[5] == 'I') && (at_commend_struct.data[6] == 'P') && (at_commend_struct.data[7] == '='))
          {
            //AT+RMIP=
            ret = strchr((const char *)at_commend_struct.data, ch);
            if(ret != NULL)
            {
              uart_write_bytes(UART_NUM_0, (ret + 1), strlen(ret));
              my_nvs_write(my_handle, "Remote", "IP", (ret + 1));
              ESP_LOGI(TAG, "Set remote IP success!\n");            
            }
          }
          else if((at_commend_struct.data[5] == 'P') && (at_commend_struct.data[6] == 'O') && (at_commend_struct.data[7] == '='))
          {
            ret = strchr((const char *)at_commend_struct.data, ch);
            if(ret != NULL)
            {
              uart_write_bytes(UART_NUM_0, (ret + 1), strlen(ret));
              my_nvs_write(my_handle, "Remote", "Port", (ret + 1));
              ESP_LOGI(TAG, "Set remote port success!\n");            
            }    
          }
        }
        else if((at_commend_struct.data[3] == 'R') && (at_commend_struct.data[4] == 'S') && (at_commend_struct.data[5] == 'T'))
        {
          esp_restart();
        }
      }
      at_commend_struct.len = 0;
      memset(at_commend_struct.data, 0, sizeof(at_commend_struct.data));
    }
    vTaskDelay(30/portTICK_RATE_MS);
  }
  vTaskDelete(NULL);
}


// void flash_read_task(void *pvParameter)
// {
//   size_t size;
// //  my_nvs_write(my_handle, MY_NAMESPACE, "key_one", "kangkang");
//   esp_err_t err = nvs_open(MY_NAMESPACE, NVS_READWRITE, &my_handle);
//   if (err != ESP_OK) 
//     ESP_LOGW(TAG, "flash open error!\n");
//   nvs_get_str(my_handle, "AT", NULL, &size);
//   char* ip_addr = malloc(size);
//   nvs_get_str(my_handle, "AT", ip_addr, &size);
//   nvs_close(my_handle);
//   while(1)
//   {
// //    uart_write_bytes(UART_NUM_0, (const char *)ip_addr, size);
//     ESP_LOGI(TAG, "%d",size);       
//     ESP_LOG_BUFFER_CHAR(TAG, ip_addr, size);
//     vTaskDelay(3000/portTICK_RATE_MS);
//   }
//   vTaskDelete(NULL);
// }


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
  eth_config("ESP32-GateWay", inet_addr("192.168.3.66"),
                              inet_addr("192.168.3.1"), inet_addr("255.255.255.0"), inet_addr("192.168.3.1"), 0);
  xTaskCreatePinnedToCore(&uart_task, "uart_task", 4 * 1024, NULL, 6, NULL, 1);
  xTaskCreatePinnedToCore(&at_commend_task, "at_commend_task", 4 * 1024, NULL, 4, NULL, 1);
  xTaskCreate(&tcp_client_task, "tcp_client_task", 8 * 1024, NULL, 8, NULL);
//  xTaskCreate(&flash_read_task, "flash_task", 4096, NULL, 2, NULL);
 // xTaskCreate(&flash_write_task, "flash_task", 4096, NULL, 1, NULL);
}
