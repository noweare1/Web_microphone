/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/*  This code is derived using WiFi softAP & station Example of esp-idf
    Note: Remember to change variable local in audio.html to chose
    whether this app is going to be used locally or remotely before
    building.
*/

#include <string.h>
#include <sys/param.h>
#include <stdint.h>
#include <esp_http_server.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <driver/i2s_std.h>
#include <driver/ledc.h>
#include <esp_timer.h>
#include <esp_mac.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_netif_net_stack.h>
#include <esp_netif.h>
#include <nvs_flash.h>
#include <lwip/inet.h>
#include <lwip/netdb.h>
#include <lwip/sockets.h>
#if IP_NAPT
#include "lwip/lwip_napt.h"
#endif
#include <lwip/err.h>
#include  <lwip/sys.h>
#include "favicon_ico.h"
#include "audio_html.h"
#include "index_ap_html.h"
#include "index_ap1_html.h"
#include "cJSON.h"

/* STA Configuration */
#define EXAMPLE_ESP_MAXIMUM_RETRY           CONFIG_ESP_MAXIMUM_STA_RETRY

#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD   WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD   WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD   WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD   WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD   WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD   WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD   WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD   WIFI_AUTH_WAPI_PSK
#endif

/* AP Configuration */
#define EXAMPLE_ESP_WIFI_AP_SSID            CONFIG_ESP_WIFI_AP_SSID
#define EXAMPLE_ESP_WIFI_AP_PASSWD          CONFIG_ESP_WIFI_AP_PASSWORD
#define EXAMPLE_ESP_WIFI_CHANNEL            CONFIG_ESP_WIFI_AP_CHANNEL
#define EXAMPLE_MAX_STA_CONN                CONFIG_ESP_MAX_STA_CONN_AP


/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define displayMsgs false

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (5) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4096) // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY          (4000) // Frequency in Hertz. Set frequency at 4 kHz

#define GREEN GPIO_NUM_17
#define RED GPIO_NUM_16
#define BLUE GPIO_NUM_4
#define DELAY 3000

#define PWM 20
//function prototypes
static esp_err_t wifi_password_handler(httpd_req_t*);
static esp_err_t root_ap_handler(httpd_req_t*);
static esp_err_t favicon_handler(httpd_req_t*);
static esp_err_t url_get_handler(httpd_req_t*);
static esp_err_t audio_handler(httpd_req_t*);
static esp_err_t echo_handler(httpd_req_t *);
static void ledc_init(void);
void init_microphone(void);

void btn_shutdown(void);

esp_netif_t* wifi_init_sta(char*, char*);
esp_netif_t*  wifi_init_softap(void);
esp_err_t nvsRead(char *ssid, char *pwd);
esp_err_t nvsWrite(char *ssid, char *pwd);
void unregister_handlers(void);
//void wifiConfig(bool dualmode, char*, char*);

static const char *TAG_AP = "WiFi SoftAP";
static const char *TAG_STA = "WiFi Sta";

static int s_retry_num = 0;
bool got_credentials = true;
const int myPort = 8080;
char ipAddress[20]="";
char buf[200];
bool dualMode=true;
bool i2s_enabled = false;
i2s_chan_handle_t rx_handle = NULL;
#define SAMPLE_SIZE (4096)
#define CONFIG_EXAMPLE_SAMPLE_RATE 8000 // use 16KHz as the sample rate
static int32_t i2s_read_buff[SAMPLE_SIZE]; // user storage for mic samples
static int16_t audio_samples[SAMPLE_SIZE]; // user storage for conversion of mic samples to dma output buffer
size_t bytes_read;

httpd_handle_t server = NULL;

static EventGroupHandle_t s_wifi_event_group;


static const httpd_uri_t wifi_uri = {
        .uri = "/wifi",
        .method = HTTP_POST,
        .handler = wifi_password_handler};
    
    static const httpd_uri_t url_uri = {
        .uri = "/url",
        .method = HTTP_GET,
        .handler = url_get_handler};

    static const httpd_uri_t root_ap_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_ap_handler};

    static const httpd_uri_t favicon_uri = {
        .uri = "/favicon.ico",
        .method = HTTP_GET,
        .handler = favicon_handler,
        .user_ctx = NULL};
        
    static const httpd_uri_t ws_uri = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = echo_handler,
        .user_ctx = NULL,
        .is_websocket = true};

    static const httpd_uri_t audio_uri = {
        .uri = "/audio",
        .method = HTTP_GET,
        .handler = audio_handler,
        .user_ctx = NULL};


void init_microphone(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    // i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_handle));

    i2s_std_config_t rx_std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(CONFIG_EXAMPLE_SAMPLE_RATE),
        /* The default mono slot is the left slot (whose 'select pin' of the PDM microphone is pulled down) */
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED, // some codecs may require mclk signal, this example doesn't need it
            .bclk = GPIO_NUM_14,
            .ws = GPIO_NUM_12,
            .dout = I2S_GPIO_UNUSED,
            .din = GPIO_NUM_33,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle, &rx_std_cfg));
    // ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));
}

static void ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t led_red = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = RED,
        .duty           = 50, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&led_red));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t led_green = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_1,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = GREEN,
        .duty           = 50, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&led_green));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t led_blue = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_2,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = BLUE,
        .duty           = 50, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&led_blue));
}

void green_led(void){
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, PWM-PWM));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0)); 

  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, PWM));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1)); 

  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_2, PWM-PWM));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_2)); 
}


void blue_led(void){
   //turn on blue status led 
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, PWM-PWM));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0));  //  // Update duty to apply the new value

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, PWM-PWM));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1));  //  // Update duty to apply the new value

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_2, PWM));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_2));  //  // Update duty to apply the new value
}


static httpd_handle_t start_webserver(void)
{
    // httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    // change config based on application
    config.max_open_sockets = 3;
    config.lru_purge_enable = true;
    config.server_port = myPort;
   
    // Start the httpd server
    ESP_LOGI(TAG_AP, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK)
    {       
      // ESP_LOGI(TAG_AP, "Registering URI handlers");
      httpd_register_uri_handler(server, &root_ap_uri);
      httpd_register_uri_handler(server, &wifi_uri);
      httpd_register_uri_handler(server, &url_uri);
      // httpd_register_err_handler(server, HTTPD_404_NOT_FOUND, http_404_error_handler);
      httpd_register_uri_handler(server, &ws_uri);
      httpd_register_uri_handler(server, &audio_uri);
      httpd_register_uri_handler(server, &favicon_uri);
  
      return server;
    }

    return NULL;
}

void btn_shutdown(void)
{  
  char ssid[25] = "test";      
  char password[25] = "test";   //this will always fail since it is not long enout
  esp_err_t rtn = ESP_FAIL;
  //stop_webserver(server);
  esp_wifi_stop();
  vTaskDelay(pdMS_TO_TICKS(1000));
  rtn = nvsWrite(ssid, password);   //write something that will fail
  if (rtn == ESP_OK) {
    ESP_LOGI(TAG_STA, "Credentials Overwritten, restarting esp32 ...");
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
  else {
    ESP_LOGE(TAG_STA, "Credentials NOT Overwritten");
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

static esp_err_t favicon_handler(httpd_req_t *req) // this is working
{
    if (req->method == HTTP_GET)
    {
        httpd_resp_set_type(req, "image/x-icon");
        httpd_resp_set_hdr(req, "Content-Encoding", "identity");
        httpd_resp_set_hdr(req, "Cache-Control", "max-age=86400");
        ESP_LOGI(TAG_STA, "Sending favicon icon");
        httpd_resp_send(req, (const char*)favicon_ico, favicon_ico_len);
    }
    return ESP_OK;
}


/* This handler handles all requests from the websocket */
static esp_err_t echo_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET)
    {
        // httpd_resp_set_type(req, "text/html");
        // httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
        ESP_LOGI(TAG_STA, "Serving audio web page");
        // httpd_resp_send(req, (const char *)index_html_gz, index_html_gz_len);
        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt;
    uint8_t *buf = NULL;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    /* Set max_len = 0 to get the frame len */
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG_STA, "httpd_ws_recv_frame failed to get frame len with %d", ret);
        return ret;
    }

    if (displayMsgs)
        ESP_LOGI(TAG_STA, "frame len is %d", ws_pkt.len);

    if (ws_pkt.len > 0)
    {   /* ws_pkt.len + 1 is for NULL termination as we are expecting a string */
        buf = (uint8_t *)calloc(1, ws_pkt.len + 1);
        if (buf == NULL)
        {
            ESP_LOGE(TAG_STA, "Failed to calloc memory for buf");
            return ESP_ERR_NO_MEM;
        }
        ws_pkt.payload = buf;
        // Set max_len = ws_pkt.len to get the frame payload
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG_STA, "httpd_ws_recv_frame failed with %d", ret);
            free(buf);
            return ret;
        }

        if (displayMsgs)
            ESP_LOGI(TAG_STA, "Got packet with message: %s", ws_pkt.payload);
    }

    if (displayMsgs)
        ESP_LOGI(TAG_STA, "Packet type: %d", ws_pkt.type);

    if ((ws_pkt.type == HTTPD_WS_TYPE_TEXT) && (strstr((char *)ws_pkt.payload, "load") != 0))
    {
        if (i2s_enabled == false)
        {
            esp_wifi_set_ps(WIFI_PS_NONE);
            i2s_channel_enable(rx_handle);
            i2s_enabled = true;
        }

        esp_err_t ret = ESP_OK;
        ws_pkt.payload = (uint8_t *)audio_samples;
        ws_pkt.final = true;
        ws_pkt.fragmented = false;
        ws_pkt.type = HTTPD_WS_TYPE_BINARY;
        int samples_read = 0;

        if (i2s_channel_read(rx_handle, (int32_t *)i2s_read_buff, SAMPLE_SIZE * sizeof(int32_t), &bytes_read, 100) == ESP_OK)
        {
            if (displayMsgs)
                ESP_LOGI(TAG_STA, "bytes read: %u", bytes_read);

            samples_read = bytes_read / sizeof(int32_t);

            for (int i = 0; i < samples_read; i++)
                audio_samples[i] = (i2s_read_buff[i] & 0xFFFFFFF0) >> 11; // convert 32bit ESP_LOGI(TAG, "bytes read: %u", bytes_read); sample to 16bit sample
        }
        else
        {
            ESP_LOGI(TAG_STA, "Read Failed!\n");
            return ESP_OK;
        }

        if (displayMsgs)
        {
            for (int i = 0; i < 4; i++)
                ESP_LOGI(TAG_STA, "%x ", audio_samples[i]);
        }

        ws_pkt.len = samples_read * 2; // bytes read,  ws_pkt.payload = (uint8_t *)i2s_samples;

        vTaskDelay(pdMS_TO_TICKS(3));

        ret = httpd_ws_send_frame(req, &ws_pkt);

        if (ret != ESP_OK)
        {
            ESP_LOGI(TAG_STA, "ERROR: %s", esp_err_to_name(ret));
            return ret;
        }

        if (displayMsgs)
            ESP_LOGI(TAG_STA, "Sent frame succesfully");

        return ret;
    }

    else if ((ws_pkt.type == HTTPD_WS_TYPE_TEXT) && (strstr((char *)ws_pkt.payload, "ping") != 0))
    {

        if (displayMsgs)
            ESP_LOGI(TAG_STA, "rcvd ping req %llu", esp_timer_get_time() / 1000);

        ws_pkt.payload = (uint8_t *)"pong";
        ret = httpd_ws_send_frame(req, &ws_pkt);
        free(buf);
    }
    else if ((ws_pkt.type == HTTPD_WS_TYPE_TEXT) && (strstr((char *)ws_pkt.payload, "stop") != 0))
    {
        if (displayMsgs)
            ESP_LOGI(TAG_STA, "rcvd stop req %llu", esp_timer_get_time() / 1000);

        if (i2s_enabled == true)
        {
            ESP_ERROR_CHECK(i2s_channel_disable(rx_handle));
            esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
            i2s_enabled = false;
        }

        free(buf);
        ret = ESP_OK;
        // ws_pkt.payload = (uint8_t *)"stop";
        // ret = httpd_ws_send_frame(req, &ws_pkt);
    }
    // if request not load, stop or ping it gets handled and echoed back here
    else
    {
        ret = httpd_ws_send_frame(req, &ws_pkt);
        free(buf);
    }
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG_STA, "httpd_ws_send_frame failed with %d", ret);
    }

    return ret;
}
// ws root station handler, serves up index.html in station mode
static esp_err_t audio_handler(httpd_req_t *req) // this is working
{
    if (req->method == HTTP_GET)
    {   unregister_handlers();
        httpd_resp_set_type(req, "text/html");
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
        ESP_LOGI(TAG_STA, "serving root webpage");
        httpd_resp_send(req, (const char *)audio_html_gz, audio_html_gz_len);
       
    }
    return ESP_OK;
}


// Used in AP mode, handles the storing of user network SSID and Password
static esp_err_t wifi_password_handler(httpd_req_t *req)
{
    char ssid[25];
    char password[25];
    esp_err_t rtn; // rtn holds an var of type esp_err_t
    
    memset(buf, 0, sizeof(buf));
    size_t recv_size = MIN(req->content_len, sizeof(buf));
    cJSON *credentials;
    httpd_resp_set_type(req, "text/plain");
    httpd_req_recv(req, buf, recv_size); // read content data from rqst
    credentials = cJSON_Parse(buf);

    strcpy(ssid, cJSON_GetObjectItem(credentials, "ssid")->valuestring);
    strcpy(password, cJSON_GetObjectItem(credentials, "pwd")->valuestring);

    ESP_LOGI(TAG_AP, "user ssid = %s",ssid);
    ESP_LOGI(TAG_AP, "user pwd = %s", password);

    // if ssid or password is empty    
    if ((strcmp(password, "") == 0) || (strcmp(ssid, "") == 0))
    {
        ESP_LOGE(TAG_AP, "SSID or Password fields is empty");
        strcpy(buf, "ERROR: SSID or Password is empty");
        httpd_resp_set_status(req, HTTPD_500); // server error
        httpd_resp_send(req, buf, strlen(buf));
        return ESP_FAIL;
    }

    ESP_LOGI(TAG_AP, "Saving creditials to NVS");
    rtn = nvsWrite(ssid, password);
    if (rtn != ESP_OK)
    {
        ESP_LOGE(TAG_AP, "ERROR: Credentials not written to NVS");
    }

    ESP_LOGI(TAG_AP, "Creditials saved to NVS success");
    strcpy(buf, "Creditials saved resetting the esp32 ...");
    httpd_resp_set_status(req, HTTPD_200);
    httpd_resp_send(req, buf, strlen(buf));
    vTaskDelay(pdMS_TO_TICKS(3000));
    //esp_wifi_deinit();
    esp_restart();
   
    return ESP_OK;
}


// NOTE : THE QUERY STRING IS THE PART AFTER THE ?
// Since this is a "GET" handler there is no body in the request so we do not use
// httpd_req_recv() function to read the body. 
// We must use http_req_get_url_query_str() instead.
static esp_err_t url_get_handler(httpd_req_t *req)
{   
    //char ssid[25], pw[25];
   
    size_t buf_len;
    buf_len = httpd_req_get_url_query_len(req) + 1;   //return len of char after the ? in the query 
    
    //gets everything after the ? in the get request ie "cmd=geturl in this case
    httpd_req_get_url_query_str(req, buf, buf_len);

    // size_t recv_size = MIN(req->content_len, sizeof(buf));  size of body content, used in post requests
    httpd_resp_set_type(req, "text/plain");
    //httpd_req_recv(req, buf, buf_len);  reads the body content which is 0 for a get request
    ESP_LOGI(TAG_AP, "buf : %s", buf);
    
    if (strstr (buf,"geturl") == NULL) {
      ESP_LOGE(TAG_AP, "HTTPD_404 : resource not found on server");
      strcpy(buf, "err: resource not found");
      httpd_resp_set_status(req, HTTPD_404); // server error
      httpd_resp_send(req, buf, strlen(buf));
      return ESP_FAIL;
    }
    
    httpd_resp_set_type(req, "text/plain"); 
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_status(req, HTTPD_200);
    
    sprintf(buf, "%s:%d/audio", ipAddress, myPort); 
    httpd_resp_send(req, buf, strlen(buf));
     
    return ESP_OK;
}


static esp_err_t root_ap_handler(httpd_req_t *req) // this is working
{
    if (req->method == HTTP_GET)
    {
        httpd_resp_set_type(req, "text/html");
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
        ESP_LOGI(TAG_AP, "Serving root ap webpage");
        if(got_credentials == false)
          httpd_resp_send(req, (const char *)index_ap_html_gz, index_ap_html_gz_len);
        else
          httpd_resp_send(req, (const char *)index_ap1_html_gz, index_ap1_html_gz_len);

    }
    return ESP_OK;
}



void unregister_handlers(void) {
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  //vTaskDelay(pdMS_TO_TICKS(1000));
  httpd_unregister_uri_handler(server, "/wifi", HTTP_POST);
  httpd_unregister_uri_handler(server, "/url", HTTP_GET);
}


//---- write user password and ssid to nvs storage ------//
esp_err_t nvsWrite(char* ssid, char* pwd)
{
    static nvs_handle_t my_handle;
    esp_err_t err = ESP_OK;
    
    ESP_LOGI(TAG_AP, "ssid is %s,  pwd is %s", ssid, pwd);

    err = nvs_open("nvs", NVS_READWRITE, &my_handle);
    if (err != ESP_OK){
        ESP_LOGE(TAG_AP, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_str(my_handle, "pwd", pwd); // const char* key, const char* value)
    if (err != ESP_OK) {
        ESP_LOGE(TAG_AP, "Error (%s) Failed writing pwd!", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_str(my_handle, "ssid",ssid); // const char* key, const char* value)
    if (err != ESP_OK) {
        ESP_LOGE(TAG_AP, "Error (%s) Failed writing ssid", esp_err_to_name(err));
        return err;
    }
   
    ESP_LOGI(TAG_AP, "Committing updates to NVS ... ");

    err = nvs_commit(my_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_AP, "Error (%s) Failed commit", esp_err_to_name(err));
        return err;
    }

    nvs_close(my_handle);
    return ESP_OK;
}


// when application is run the first thing we want to do is check nvs for wifi credentials
esp_err_t nvsRead(char *ssid, char *pwd)
{
    // the keys will always be "ssid" and "pwd" 
    nvs_handle_t my_handle;
    esp_err_t rtn = ESP_OK;

    rtn = nvs_open("nvs", NVS_READWRITE, &my_handle);
    if (rtn != ESP_OK)    {
        ESP_LOGE(TAG_AP, "Error (%s) could not open an NVS handle", esp_err_to_name(rtn));
        return rtn;
    }

    ESP_LOGI(TAG_AP, "NVS opened, reading from NVS partition");

    size_t required_size;

    // First call to get the required size for the string
    rtn = nvs_get_str(my_handle, "pwd", NULL, &required_size);
    // check required size against pwd string size if necessary
    if (rtn != ESP_OK)  {
        ESP_LOGE(TAG_AP, "Error (%s) could not get required size for pwd", esp_err_to_name(rtn));
        nvs_close(my_handle);
        return rtn;
    }

    rtn = nvs_get_str(my_handle, "pwd", pwd, &required_size);
    if (rtn != ESP_OK)  { // Second call to get the string
        ESP_LOGE(TAG_AP, "Error (%s) could not read pwd from NVS", esp_err_to_name(rtn));
        nvs_close(my_handle);
        return rtn;
    }

    // get the ssid from NVS   
    rtn = nvs_get_str(my_handle, "ssid", NULL, &required_size);
    // check required size against pwd string size if necessary
    if (rtn != ESP_OK) {
        ESP_LOGE(TAG_AP, "Error (%s) could not get required size for ssid", esp_err_to_name(rtn));
        nvs_close(my_handle);
        return rtn;
    }

    rtn = nvs_get_str(my_handle, "ssid", ssid, &required_size);
    if (rtn != ESP_OK) { // Second call to get the string
        ESP_LOGE(TAG_AP, "Error (%s) could not read pw from NVS", esp_err_to_name(rtn));
        nvs_close(my_handle);
        return rtn;
    }

    // need a way to check the strings
    ESP_LOGI(TAG_AP, "SSID %s PWD %s read from NVS", ssid, pwd);

    // series of tests on both strings
    if (strlen(pwd) < 8)  {
        ESP_LOGE(TAG_AP, "error password less than 8");
        return ESP_FAIL;
    }
    if (strlen(pwd) > 63) {
        ESP_LOGE(TAG_AP, "error password > 63");
        return ESP_FAIL;
    }
    if (strlen(ssid) > 32)  {
        ESP_LOGE(TAG_AP, "error ssid > 32");
        return ESP_FAIL;
    }
    for (int i = 0; i < (int)strlen(pwd); i++) {
        if ((pwd[i] < 33) || (pwd[i] > 122))
        {
            ESP_LOGE(TAG_AP, "%s failed with %c not within range\n", pwd, pwd[i]);
            rtn = ESP_FAIL;
            break;
        }
    }

    for (int i = 0; i < (int)strlen(ssid); i++)  {
        if ((ssid[i] < 33) || (ssid[i] > 122)) {
            ESP_LOGE(TAG_AP, "%s failed with %c not within range\n", ssid, ssid[i]);
            rtn = ESP_FAIL;
            break;
        }
    }

    // close NVS handle
    nvs_close(my_handle);

    return rtn;
} 


static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *) event_data;
        ESP_LOGI(TAG_AP, "Station "MACSTR" joined, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *) event_data;
        ESP_LOGI(TAG_AP, "Station "MACSTR" left, AID=%d, reason:%d",
                 MAC2STR(event->mac), event->aid, event->reason);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG_STA, "Station started");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG_STA, "Station disconnected, retrying in 3 secs");
        if (s_retry_num <= EXAMPLE_ESP_MAXIMUM_RETRY) {
          vTaskDelay(pdMS_TO_TICKS(3000));
          s_retry_num++;
          //esp_wifi_connect();
          esp_wifi_start();   //issue a start instead of another wifi connect
        }
        else {
          ESP_LOGI(TAG_STA, "Station retries over limit, resetting credentials");
          btn_shutdown(); //we write creds that fail in order to get current creds from user
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG_STA, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        sprintf(ipAddress, IPSTR, IP2STR(&event->ip_info.ip)); 
        green_led();  
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}


static void http_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
   // httpd_handle_t *server = (httpd_handle_t *)arg;  //the handler was configed to pass the server to the argument
   
    switch (event_id)
    {
    case HTTP_SERVER_EVENT_START:
        ESP_LOGI(TAG_STA, "HTTP Server Started");
        break;

    case HTTP_SERVER_EVENT_DISCONNECTED: 
       // ret = stop_webserver(*server);
       // ESP_LOGI(TAG, "HTTP Server disconnected, stopping Server");
       // if (ret == ESP_OK)
       //    *server = NULL;
       // else
            ESP_LOGI(TAG_STA, "client disconnected from server");
        break;

    case HTTP_SERVER_EVENT_STOP:
        ESP_LOGI(TAG_STA, "HTTP Server Stopped");
        break;

    case HTTP_SERVER_EVENT_ERROR:
        ESP_LOGI(TAG_STA, "HTTP Server Error Occurred");
        break;

    case HTTP_SERVER_EVENT_ON_CONNECTED:
        ESP_LOGI(TAG_STA, "HTTP Server connected to client");
        break;

    case HTTP_SERVER_EVENT_ON_HEADER:
        ESP_LOGI(TAG_STA, "Server recieved headers");
        break;

    case HTTP_SERVER_EVENT_ON_DATA:
        ESP_LOGI(TAG_STA, "Server got data from client");
        break;
    }
}


/* Initialize soft AP */
esp_netif_t *wifi_init_softap(void)
{
    esp_netif_t *esp_netif_ap = esp_netif_create_default_wifi_ap();

    wifi_config_t wifi_ap_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_AP_SSID,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_AP_SSID),
            .channel = EXAMPLE_ESP_WIFI_CHANNEL,
            .password = EXAMPLE_ESP_WIFI_AP_PASSWD,
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };

    if (strlen(EXAMPLE_ESP_WIFI_AP_PASSWD) == 0) {
        wifi_ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_ap_config));

    ESP_LOGI(TAG_AP, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             EXAMPLE_ESP_WIFI_AP_SSID, EXAMPLE_ESP_WIFI_AP_PASSWD, EXAMPLE_ESP_WIFI_CHANNEL);

    blue_led();

    return esp_netif_ap;
}

/* Initialize wifi station */
esp_netif_t *wifi_init_sta(char* ssid, char* password)
{
    esp_netif_t *esp_netif_sta = esp_netif_create_default_wifi_sta();

    wifi_config_t wifi_sta_config = {
        .sta = {
           // .ssid = EXAMPLE_ESP_WIFI_STA_SSID,
           // .password = EXAMPLE_ESP_WIFI_STA_PASSWD,
            .scan_method = WIFI_ALL_CHANNEL_SCAN,
            .failure_retry_cnt = EXAMPLE_ESP_MAXIMUM_RETRY,
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (password len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
            * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };
    
    //strcpy((char *)wifi_sta_config.sta.ssid, "boulderhill"); //copy user supplied credentials to the wifi configuration 
    //strcpy((char *)wifi_sta_config.sta.password, "wideflower594");

    strcpy((char *)wifi_sta_config.sta.ssid, ssid); //copy user supplied credentials to the wifi configuration 
    strcpy((char *)wifi_sta_config.sta.password, password);

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_sta_config) );

    ESP_LOGI(TAG_STA, "wifi_init_sta finished.");

    return esp_netif_sta;
}




void app_main(void)
{ 
    char ssid[25], password[25]; // creds for user home network
    char* myssid = ssid;
    esp_netif_t *esp_netif_sta;
    esp_netif_t *esp_netif_ap;
    char* mypassword = password;
   
    esp_err_t rtn = ESP_OK;
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Initialize event group */
    s_wifi_event_group = xEventGroupCreate();

    /* Register Event handler */
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                    ESP_EVENT_ANY_ID,
                    &wifi_event_handler,
                    NULL,
                    NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                    IP_EVENT_STA_GOT_IP,
                    &wifi_event_handler,
                    NULL,
                    NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(ESP_HTTP_SERVER_EVENT,   //HTTP SERVER base
                                               ESP_EVENT_ANY_ID,        //only this event will cause calling this handler
                                               &http_event_handler,   //has a sepeate handler than WIFI, IP bases
                                               &server));


    /*Initialize WiFi */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ledc_init();
    rtn = nvsRead(myssid, mypassword);  
    //rtn = ESP_OK;

    if (rtn != ESP_OK)  //if creds fail start up the ap
    { 
      ESP_LOGI(TAG_AP, "*** Credentials NOT FOUND ***");
      got_credentials = false;
      
      ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
      esp_netif_ap = wifi_init_softap();  
      ESP_ERROR_CHECK(esp_wifi_start() ); 
      start_webserver(); 
      
    }
    //this else is run once after bootup when we have credentials
    //once the user connects to the audio.html we change to station mode
    //which drops esp32 AP mode. We do not have to stop/start wif or 
    //do anything else
    else {
  
    ESP_LOGI(TAG_STA, "*** Credentials FOUND ***");
    got_credentials = true;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));

    //Initialize AP 
    ESP_LOGI(TAG_AP, "ESP_WIFI_MODE_AP");
    esp_netif_ap = wifi_init_softap();

    // Initialize STA 
    ESP_LOGI(TAG_STA, "ESP_WIFI_MODE_STA");
    esp_netif_sta = wifi_init_sta(myssid, mypassword);

    // Start WiFi 
    ESP_ERROR_CHECK(esp_wifi_start() );
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Wait until either the connection is established (WIFI_CONNECTED_BIT) or
    // connection failed for the maximum number of re-tries (WIFI_FAIL_BIT).
    // The bits are set by event_handler() (see above)
     
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    // xEventGroupWaitBits() returns the bits before the call returned,
    // hence we can test which event actually happened. 

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG_STA, "connected to ap SSID:%s password:%s",
                 myssid, mypassword);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG_STA, "Failed to connect to SSID:%s, password:%s",
                 myssid, mypassword);
    } else {
        ESP_LOGE(TAG_STA, "UNEXPECTED EVENT");
        return;
    }

    /* Set sta as the default interface */
    esp_netif_set_default_netif(esp_netif_sta);

    /* Enable napt on the AP netif */
    if (esp_netif_napt_enable(esp_netif_ap) != ESP_OK) {
        ESP_LOGE(TAG_STA, "NAPT not enabled on the netif: %p", esp_netif_ap);
    }
    init_microphone();
    start_webserver();
}
}