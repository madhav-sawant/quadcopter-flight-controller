#include "webserver.h"
#include "../adc/adc.h"
#include "../config/config.h"

#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "web";

#define WIFI_SSID "QuadPID"
#define WIFI_PASS "12345678"

static httpd_handle_t server = NULL;
static bool server_running = false;

// Minimal HTML - no CSS, just functional
static const char *HTML_PAGE =
    "<html><head><title>PID</title></head><body>"
    "<h2>QuadPID - Bat: %d mV</h2>"
    "<form method=POST action=/s>"
    "<b>Rate Roll</b> P:<input name=rp value=%.2f size=4> "
    "I:<input name=ri value=%.2f size=4> "
    "D:<input name=rd value=%.2f size=4><br>"
    "<b>Rate Pitch</b> P:<input name=pp value=%.2f size=4> "
    "I:<input name=pi value=%.2f size=4> "
    "D:<input name=pd value=%.2f size=4><br>"
    "<b>Rate Yaw</b> P:<input name=yp value=%.2f size=4> "
    "I:<input name=yi value=%.2f size=4> "
    "D:<input name=yd value=%.2f size=4><br>"
    "<b>Angle</b> Roll P:<input name=arp value=%.2f size=4> "
    "Pitch P:<input name=app value=%.2f size=4><br><br>"
    "<input type=submit value=SAVE></form>"
    "<form method=POST action=/r><input type=submit value=RESET></form>"
    "</body></html>";

static float parse_float(const char *buf, const char *key, float def) {
  char k[16];
  snprintf(k, sizeof(k), "%s=", key);
  char *p = strstr(buf, k);
  if (!p)
    return def;
  return strtof(p + strlen(k), NULL);
}

static esp_err_t get_handler(httpd_req_t *req) {
  char *html = malloc(2048);
  if (!html)
    return ESP_FAIL;

  snprintf(html, 2048, HTML_PAGE, adc_read_battery_voltg(), sys_cfg.roll_kp,
           sys_cfg.roll_ki, sys_cfg.roll_kd, sys_cfg.pitch_kp, sys_cfg.pitch_ki,
           sys_cfg.pitch_kd, sys_cfg.yaw_kp, sys_cfg.yaw_ki, sys_cfg.yaw_kd,
           sys_cfg.angle_roll_kp, sys_cfg.angle_pitch_kp);

  httpd_resp_send(req, html, strlen(html));
  free(html);
  return ESP_OK;
}

static esp_err_t save_handler(httpd_req_t *req) {
  char buf[256];
  int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
  if (len <= 0)
    return ESP_FAIL;
  buf[len] = 0;

  sys_cfg.roll_kp = parse_float(buf, "rp", sys_cfg.roll_kp);
  sys_cfg.roll_ki = parse_float(buf, "ri", sys_cfg.roll_ki);
  sys_cfg.roll_kd = parse_float(buf, "rd", sys_cfg.roll_kd);
  sys_cfg.pitch_kp = parse_float(buf, "pp", sys_cfg.pitch_kp);
  sys_cfg.pitch_ki = parse_float(buf, "pi", sys_cfg.pitch_ki);
  sys_cfg.pitch_kd = parse_float(buf, "pd", sys_cfg.pitch_kd);
  sys_cfg.yaw_kp = parse_float(buf, "yp", sys_cfg.yaw_kp);
  sys_cfg.yaw_ki = parse_float(buf, "yi", sys_cfg.yaw_ki);
  sys_cfg.yaw_kd = parse_float(buf, "yd", sys_cfg.yaw_kd);
  sys_cfg.angle_roll_kp = parse_float(buf, "arp", sys_cfg.angle_roll_kp);
  sys_cfg.angle_pitch_kp = parse_float(buf, "app", sys_cfg.angle_pitch_kp);

  config_save_to_nvs();

  httpd_resp_set_status(req, "303 See Other");
  httpd_resp_set_hdr(req, "Location", "/");
  httpd_resp_send(req, NULL, 0);
  return ESP_OK;
}

static esp_err_t reset_handler(httpd_req_t *req) {
  config_load_defaults();
  config_save_to_nvs();
  httpd_resp_set_status(req, "303 See Other");
  httpd_resp_set_hdr(req, "Location", "/");
  httpd_resp_send(req, NULL, 0);
  return ESP_OK;
}

static void start_server(void) {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.core_id = 0;
  cfg.stack_size = 4096;

  if (httpd_start(&server, &cfg) == ESP_OK) {
    httpd_uri_t get = {.uri = "/", .method = HTTP_GET, .handler = get_handler};
    httpd_uri_t save = {
        .uri = "/s", .method = HTTP_POST, .handler = save_handler};
    httpd_uri_t reset = {
        .uri = "/r", .method = HTTP_POST, .handler = reset_handler};
    httpd_register_uri_handler(server, &get);
    httpd_register_uri_handler(server, &save);
    httpd_register_uri_handler(server, &reset);
    server_running = true;
  }
}

static void wifi_handler(void *arg, esp_event_base_t base, int32_t id,
                         void *data) {
  if (id == WIFI_EVENT_AP_START)
    start_server();
}

void webserver_init(void) {
  esp_netif_init();
  esp_event_loop_create_default();
  esp_netif_create_default_wifi_ap();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);
  esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                      &wifi_handler, NULL, NULL);

  wifi_config_t wcfg = {
      .ap =
          {
              .ssid = WIFI_SSID,
              .ssid_len = strlen(WIFI_SSID),
              .channel = 1,
              .password = WIFI_PASS,
              .max_connection = 2,
              .authmode = WIFI_AUTH_WPA_WPA2_PSK,
          },
  };

  esp_wifi_set_mode(WIFI_MODE_AP);
  esp_wifi_set_config(WIFI_IF_AP, &wcfg);
  esp_wifi_start();
}

bool webserver_is_running(void) { return server_running; }
