#include "svc_ota_esp32.h"

#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>

#define SVC_OTA_WIFI_CONNECTED_BIT BIT0
#define SVC_OTA_URL_CAPACITY       96u
#define SVC_OTA_DISCOVERY_REQUEST  "APP_OTA_DISCOVER_V1"
#define SVC_OTA_DISCOVERY_RESPONSE "APP_OTA_SERVER_V1"
#define SVC_OTA_DISCOVERY_ATTEMPTS 3u

static const char *TAG = "svc_ota";

static EventGroupHandle_t g_wifi_events;
static esp_netif_t *g_wifi_netif;
static bool g_initialized;
static bool g_wifi_started;

static bool svc_ota_esp32_initialize(void);
static void svc_ota_esp32_wifi_event(
    void *argument,
    esp_event_base_t event_base,
    int32_t event_id,
    void *event_data);
static uint8_t svc_ota_esp32_progress_percent(
    esp_https_ota_handle_t handle);

svc_ota_esp32_result_t svc_ota_esp32_connect(
    const char *ssid,
    const char *password,
    uint32_t timeout_ms)
{
    if ((ssid == NULL) || (password == NULL) || (ssid[0] == '\0') ||
        (timeout_ms == 0u) || !svc_ota_esp32_initialize())
    {
        return SVC_OTA_ESP32_ERROR_INIT;
    }

    if (g_wifi_started)
    {
        svc_ota_esp32_disconnect();
    }

    wifi_config_t wifi_config = {0};

    if ((snprintf((char *)wifi_config.sta.ssid,
                  sizeof(wifi_config.sta.ssid),
                  "%s",
                  ssid) >= (int)sizeof(wifi_config.sta.ssid)) ||
        (snprintf((char *)wifi_config.sta.password,
                  sizeof(wifi_config.sta.password),
                  "%s",
                  password) >= (int)sizeof(wifi_config.sta.password)))
    {
        return SVC_OTA_ESP32_ERROR_INIT;
    }

    wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
    wifi_config.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;
    xEventGroupClearBits(g_wifi_events, SVC_OTA_WIFI_CONNECTED_BIT);

    if ((esp_wifi_set_storage(WIFI_STORAGE_RAM) != ESP_OK) ||
        (esp_wifi_set_mode(WIFI_MODE_STA) != ESP_OK) ||
        (esp_wifi_set_config(WIFI_IF_STA, &wifi_config) != ESP_OK) ||
        (esp_wifi_start() != ESP_OK))
    {
        return SVC_OTA_ESP32_ERROR_WIFI;
    }

    g_wifi_started = true;

    if (esp_wifi_connect() != ESP_OK)
    {
        svc_ota_esp32_disconnect();
        return SVC_OTA_ESP32_ERROR_WIFI;
    }

    const EventBits_t bits = xEventGroupWaitBits(
        g_wifi_events,
        SVC_OTA_WIFI_CONNECTED_BIT,
        pdFALSE,
        pdTRUE,
        pdMS_TO_TICKS(timeout_ms));

    if ((bits & SVC_OTA_WIFI_CONNECTED_BIT) == 0u)
    {
        svc_ota_esp32_disconnect();
        return SVC_OTA_ESP32_ERROR_WIFI;
    }

    esp_wifi_set_ps(WIFI_PS_NONE);
    return SVC_OTA_ESP32_OK;
}

svc_ota_esp32_result_t svc_ota_esp32_discover_server(
    uint16_t discovery_port,
    uint32_t timeout_ms,
    char *server_address,
    size_t address_capacity)
{
    if (!svc_ota_esp32_is_connected() || (discovery_port == 0u) ||
        (timeout_ms == 0u) || (server_address == NULL) ||
        (address_capacity == 0u))
    {
        return SVC_OTA_ESP32_ERROR_INIT;
    }

    server_address[0] = '\0';

    esp_netif_ip_info_t ip_info;

    if (esp_netif_get_ip_info(g_wifi_netif, &ip_info) != ESP_OK)
    {
        return SVC_OTA_ESP32_ERROR_WIFI;
    }

    const int socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    if (socket_fd < 0)
    {
        return SVC_OTA_ESP32_ERROR_DISCOVERY;
    }

    const int broadcast_enabled = 1;
    const uint32_t attempt_timeout_ms =
        (timeout_ms + SVC_OTA_DISCOVERY_ATTEMPTS - 1u) /
        SVC_OTA_DISCOVERY_ATTEMPTS;
    const struct timeval receive_timeout =
    {
        .tv_sec = (time_t)(attempt_timeout_ms / 1000u),
        .tv_usec =
            (suseconds_t)((attempt_timeout_ms % 1000u) * 1000u),
    };

    if ((setsockopt(socket_fd,
                    SOL_SOCKET,
                    SO_BROADCAST,
                    &broadcast_enabled,
                    sizeof(broadcast_enabled)) < 0) ||
        (setsockopt(socket_fd,
                    SOL_SOCKET,
                    SO_RCVTIMEO,
                    &receive_timeout,
                    sizeof(receive_timeout)) < 0))
    {
        close(socket_fd);
        return SVC_OTA_ESP32_ERROR_DISCOVERY;
    }

    struct sockaddr_in destination =
    {
        .sin_family = AF_INET,
        .sin_port = htons(discovery_port),
        .sin_addr.s_addr = (ip_info.ip.addr & ip_info.netmask.addr) |
                            ~ip_info.netmask.addr,
    };
    const char request[] = SVC_OTA_DISCOVERY_REQUEST;
    const size_t expected_length = strlen(SVC_OTA_DISCOVERY_RESPONSE);

    for (uint32_t attempt = 0u;
         attempt < SVC_OTA_DISCOVERY_ATTEMPTS;
         attempt++)
    {
        if (sendto(socket_fd,
                   request,
                   sizeof(request) - 1u,
                   0,
                   (const struct sockaddr *)&destination,
                   sizeof(destination)) < 0)
        {
            break;
        }

        char response[32];
        struct sockaddr_in source;
        socklen_t source_length = sizeof(source);
        const int received = recvfrom(socket_fd,
                                      response,
                                      sizeof(response),
                                      0,
                                      (struct sockaddr *)&source,
                                      &source_length);

        if ((received == (int)expected_length) &&
            (memcmp(response,
                    SVC_OTA_DISCOVERY_RESPONSE,
                    expected_length) == 0) &&
            (inet_ntop(AF_INET,
                       &source.sin_addr,
                       server_address,
                       address_capacity) != NULL))
        {
            close(socket_fd);
            ESP_LOGI(TAG, "OTA server discovered at %s", server_address);
            return SVC_OTA_ESP32_OK;
        }
    }

    close(socket_fd);
    return SVC_OTA_ESP32_ERROR_DISCOVERY;
}

svc_ota_esp32_result_t svc_ota_esp32_install_from_server(
    const char *server_address,
    uint16_t server_port,
    const char *firmware_path,
    uint32_t timeout_ms,
    svc_ota_esp32_progress_fn progress_fn,
    void *progress_context)
{
    if (!svc_ota_esp32_is_connected() || (server_address == NULL) ||
        (server_address[0] == '\0') || (server_port == 0u) ||
        (firmware_path == NULL) ||
        (firmware_path[0] != '/') || (timeout_ms == 0u))
    {
        return SVC_OTA_ESP32_ERROR_INIT;
    }

    char url[SVC_OTA_URL_CAPACITY];
    const int url_length = snprintf(
        url,
        sizeof(url),
        "http://%s:%u%s",
        server_address,
        (unsigned int)server_port,
        firmware_path);

    if ((url_length < 0) || (url_length >= (int)sizeof(url)))
    {
        return SVC_OTA_ESP32_ERROR_INIT;
    }

    ESP_LOGI(TAG, "downloading from %s", url);

    esp_http_client_config_t http_config =
    {
        .url = url,
        .transport_type = HTTP_TRANSPORT_OVER_TCP,
        .timeout_ms = (int)timeout_ms,
        .keep_alive_enable = true,
    };
    esp_https_ota_config_t ota_config =
    {
        .http_config = &http_config,
    };
    esp_https_ota_handle_t ota_handle = NULL;
    esp_err_t error = esp_https_ota_begin(&ota_config, &ota_handle);

    if (error != ESP_OK)
    {
        ESP_LOGE(TAG, "OTA begin failed: %s", esp_err_to_name(error));
        return SVC_OTA_ESP32_ERROR_DOWNLOAD;
    }

    esp_app_desc_t image_description;
    error = esp_https_ota_get_img_desc(ota_handle, &image_description);

    if (error != ESP_OK)
    {
        ESP_LOGE(TAG, "image header failed: %s", esp_err_to_name(error));
        esp_https_ota_abort(ota_handle);
        return SVC_OTA_ESP32_ERROR_IMAGE;
    }

    if (progress_fn != NULL)
    {
        progress_fn(0u, progress_context);
    }

    do
    {
        error = esp_https_ota_perform(ota_handle);

        if (progress_fn != NULL)
        {
            progress_fn(svc_ota_esp32_progress_percent(ota_handle),
                        progress_context);
        }
    }
    while (error == ESP_ERR_HTTPS_OTA_IN_PROGRESS);

    if ((error != ESP_OK) ||
        !esp_https_ota_is_complete_data_received(ota_handle))
    {
        ESP_LOGE(TAG, "OTA download failed: %s", esp_err_to_name(error));
        esp_https_ota_abort(ota_handle);
        return SVC_OTA_ESP32_ERROR_DOWNLOAD;
    }

    error = esp_https_ota_finish(ota_handle);

    if (error != ESP_OK)
    {
        ESP_LOGE(TAG, "OTA image validation failed: %s",
                 esp_err_to_name(error));
        return SVC_OTA_ESP32_ERROR_IMAGE;
    }

    if (progress_fn != NULL)
    {
        progress_fn(100u, progress_context);
    }

    return SVC_OTA_ESP32_OK;
}

void svc_ota_esp32_disconnect(void)
{
    if (!g_initialized || !g_wifi_started)
    {
        return;
    }

    (void)esp_wifi_disconnect();
    (void)esp_wifi_stop();
    xEventGroupClearBits(g_wifi_events, SVC_OTA_WIFI_CONNECTED_BIT);
    g_wifi_started = false;
}

bool svc_ota_esp32_is_connected(void)
{
    return g_initialized && g_wifi_started &&
        ((xEventGroupGetBits(g_wifi_events) &
          SVC_OTA_WIFI_CONNECTED_BIT) != 0u);
}

bool svc_ota_esp32_get_ipv4(char *address, size_t capacity)
{
    if ((address == NULL) || (capacity == 0u))
    {
        return false;
    }

    address[0] = '\0';

    if (!svc_ota_esp32_is_connected())
    {
        return false;
    }

    esp_netif_ip_info_t ip_info;

    if (esp_netif_get_ip_info(g_wifi_netif, &ip_info) != ESP_OK)
    {
        return false;
    }

    const int length = snprintf(address,
                                capacity,
                                IPSTR,
                                IP2STR(&ip_info.ip));
    return (length > 0) && (length < (int)capacity);
}

bool svc_ota_esp32_confirm_running_image(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t state;

    if ((running == NULL) ||
        (esp_ota_get_state_partition(running, &state) != ESP_OK) ||
        (state != ESP_OTA_IMG_PENDING_VERIFY))
    {
        return true;
    }

    return esp_ota_mark_app_valid_cancel_rollback() == ESP_OK;
}

static bool svc_ota_esp32_initialize(void)
{
    if (g_initialized)
    {
        return true;
    }

    esp_err_t error = nvs_flash_init();

    if ((error == ESP_ERR_NVS_NO_FREE_PAGES) ||
        (error == ESP_ERR_NVS_NEW_VERSION_FOUND))
    {
        error = nvs_flash_erase();

        if (error == ESP_OK)
        {
            error = nvs_flash_init();
        }
    }

    if ((error != ESP_OK) || (esp_netif_init() != ESP_OK))
    {
        return false;
    }

    error = esp_event_loop_create_default();

    if ((error != ESP_OK) && (error != ESP_ERR_INVALID_STATE))
    {
        return false;
    }

    g_wifi_events = xEventGroupCreate();
    g_wifi_netif = esp_netif_create_default_wifi_sta();

    if ((g_wifi_events == NULL) || (g_wifi_netif == NULL))
    {
        return false;
    }

    wifi_init_config_t wifi_init = WIFI_INIT_CONFIG_DEFAULT();

    if ((esp_wifi_init(&wifi_init) != ESP_OK) ||
        (esp_event_handler_register(WIFI_EVENT,
                                    ESP_EVENT_ANY_ID,
                                    svc_ota_esp32_wifi_event,
                                    NULL) != ESP_OK) ||
        (esp_event_handler_register(IP_EVENT,
                                    IP_EVENT_STA_GOT_IP,
                                    svc_ota_esp32_wifi_event,
                                    NULL) != ESP_OK))
    {
        return false;
    }

    g_initialized = true;
    return true;
}

static void svc_ota_esp32_wifi_event(
    void *argument,
    esp_event_base_t event_base,
    int32_t event_id,
    void *event_data)
{
    (void)argument;
    (void)event_data;

    if ((event_base == IP_EVENT) && (event_id == IP_EVENT_STA_GOT_IP))
    {
        xEventGroupSetBits(g_wifi_events, SVC_OTA_WIFI_CONNECTED_BIT);
    }
    else if ((event_base == WIFI_EVENT) &&
             (event_id == WIFI_EVENT_STA_DISCONNECTED))
    {
        xEventGroupClearBits(g_wifi_events,
                             SVC_OTA_WIFI_CONNECTED_BIT);
    }
}

static uint8_t svc_ota_esp32_progress_percent(
    esp_https_ota_handle_t handle)
{
    const int image_size = esp_https_ota_get_image_size(handle);
    const int bytes_read = esp_https_ota_get_image_len_read(handle);

    if ((image_size <= 0) || (bytes_read <= 0))
    {
        return 0u;
    }

    const int percent = (bytes_read * 100) / image_size;
    return (percent >= 100) ? 99u : (uint8_t)percent;
}
