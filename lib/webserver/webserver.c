#include "webserver.h"
#include "../adc/adc.h"
#include "../blackbox/blackbox.h"
#include "../config/config.h"
#include "../imu/imu.h"
#include "../rate_control/rate_control.h"

#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// External system state from main.c
extern bool system_armed;

#define WIFI_SSID "QuadPID"
#define WIFI_PASS "12345678"

static httpd_handle_t server = NULL;

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
    "I:<input name=ari value=%.2f size=4> "
    "D:<input name=ard value=%.2f size=4><br>"
    "Pitch P:<input name=app value=%.2f size=4> "
    "I:<input name=api value=%.2f size=4> "
    "D:<input name=apd value=%.2f size=4><br><br>"
    "<input type=submit value=SAVE></form>"
    "<form method=POST action=/r><input type=submit value=RESET></form>"
    "<hr><a href=/blackbox>Download Blackbox CSV</a> | "
    "<form style='display:inline' method=POST action=/blackbox/clear>"
    "<input type=submit value='Clear Blackbox'></form> | "
    "<form style='display:inline' method=POST action=/calibrate>"
    "<input type=submit value='Recalibrate IMU'></form>"
    "<p style='color:green'><b>%s</b></p>"
    "<p style='color:red'><b>%s</b></p>"
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

  const char *msg = "";
  const char *err_msg = "";
  char query[64];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
    if (strstr(query, "msg=saved"))
      msg = "Values Inserted into PID Controller!";
    else if (strstr(query, "msg=reset"))
      msg = "Values Reset to Defaults!";
    else if (strstr(query, "msg=calibrated"))
      msg = "IMU Calibrated & Saved!";
    else if (strstr(query, "err=armed"))
      err_msg = "ERROR: Disarm first before changing settings!";
  }

  snprintf(html, 2048, HTML_PAGE, adc_read_battery_voltg(), sys_cfg.roll_kp,
           sys_cfg.roll_ki, sys_cfg.roll_kd, sys_cfg.pitch_kp, sys_cfg.pitch_ki,
           sys_cfg.pitch_kd, sys_cfg.yaw_kp, sys_cfg.yaw_ki, sys_cfg.yaw_kd,
           sys_cfg.angle_roll_kp, sys_cfg.angle_roll_ki, sys_cfg.angle_roll_kd,
           sys_cfg.angle_pitch_kp, sys_cfg.angle_pitch_ki,
           sys_cfg.angle_pitch_kd, msg, err_msg);

  httpd_resp_send(req, html, strlen(html));
  free(html);
  return ESP_OK;
}

static esp_err_t save_handler(httpd_req_t *req) {
  // Only allow saving when disarmed
  if (system_armed) {
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/?err=armed");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
  }

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
  sys_cfg.angle_roll_ki = parse_float(buf, "ari", sys_cfg.angle_roll_ki);
  sys_cfg.angle_roll_kd = parse_float(buf, "ard", sys_cfg.angle_roll_kd);
  sys_cfg.angle_pitch_kp = parse_float(buf, "app", sys_cfg.angle_pitch_kp);
  sys_cfg.angle_pitch_ki = parse_float(buf, "api", sys_cfg.angle_pitch_ki);
  sys_cfg.angle_pitch_kd = parse_float(buf, "apd", sys_cfg.angle_pitch_kd);

  config_save_to_nvs();
  rate_control_init(); // Reload PID gains into controllers

  httpd_resp_set_status(req, "303 See Other");
  httpd_resp_set_hdr(req, "Location", "/?msg=saved");
  httpd_resp_send(req, NULL, 0);
  return ESP_OK;
}

static esp_err_t reset_handler(httpd_req_t *req) {
  // Only allow reset when disarmed
  if (system_armed) {
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/?err=armed");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
  }

  config_load_defaults();
  config_save_to_nvs();
  rate_control_init(); // Reload PID gains into controllers
  httpd_resp_set_status(req, "303 See Other");
  httpd_resp_set_hdr(req, "Location", "/?msg=reset");
  httpd_resp_send(req, NULL, 0);
  return ESP_OK;
}

// Blackbox CSV download handler
static esp_err_t blackbox_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/csv");
  httpd_resp_set_hdr(req, "Content-Disposition",
                     "attachment; filename=blackbox.csv");

  // Send CSV header
  const char *header = "time_ms,gyro_x,gyro_y,gyro_z,roll,pitch,pid_roll,pid_"
                       "pitch,pid_yaw,m1,m2,m3,m4,throttle,flags\n";
  httpd_resp_sendstr_chunk(req, header);

  // Send each entry
  uint16_t count = blackbox_get_count();
  char line[256];

  for (uint16_t i = 0; i < count; i++) {
    const blackbox_entry_t *e = blackbox_get_entry(i);
    if (e) {
      snprintf(
          line, sizeof(line),
          "%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%u,%u,%u,%u,%u,%u\n",
          (unsigned long)e->timestamp_ms, e->gyro_x, e->gyro_y, e->gyro_z,
          e->angle_roll, e->angle_pitch, e->pid_roll, e->pid_pitch, e->pid_yaw,
          e->motor[0], e->motor[1], e->motor[2], e->motor[3], e->throttle,
          e->flags);
      httpd_resp_sendstr_chunk(req, line);
    }
  }

  // End chunked response
  httpd_resp_sendstr_chunk(req, NULL);
  return ESP_OK;
}

// Blackbox clear handler
static esp_err_t blackbox_clear_handler(httpd_req_t *req) {
  blackbox_clear();
  httpd_resp_set_status(req, "303 See Other");
  httpd_resp_set_hdr(req, "Location", "/");
  httpd_resp_send(req, NULL, 0);
  return ESP_OK;
}

// IMU Calibration handler
static esp_err_t calibrate_handler(httpd_req_t *req) {
  // Only allow calibration when disarmed
  if (system_armed) {
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/?err=armed");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
  }

  // Turn LED ON during calibration
  gpio_set_level(2, 1);

  imu_calibrate_gyro();
  imu_calibrate_accel();
  imu_calibration_save_to_nvs();

  // Turn LED OFF after calibration
  gpio_set_level(2, 0);

  httpd_resp_set_status(req, "303 See Other");
  httpd_resp_set_hdr(req, "Location", "/?msg=calibrated");
  httpd_resp_send(req, NULL, 0);
  return ESP_OK;
}

static void start_server(void) {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.core_id = 0;
  cfg.stack_size = 8192; // Increased for blackbox CSV
  cfg.max_uri_handlers = 8;

  if (httpd_start(&server, &cfg) == ESP_OK) {
    httpd_uri_t get = {.uri = "/", .method = HTTP_GET, .handler = get_handler};
    httpd_uri_t save = {
        .uri = "/s", .method = HTTP_POST, .handler = save_handler};
    httpd_uri_t reset = {
        .uri = "/r", .method = HTTP_POST, .handler = reset_handler};
    httpd_uri_t blackbox = {
        .uri = "/blackbox", .method = HTTP_GET, .handler = blackbox_handler};
    httpd_uri_t blackbox_clr = {.uri = "/blackbox/clear",
                                .method = HTTP_POST,
                                .handler = blackbox_clear_handler};

    httpd_register_uri_handler(server, &get);
    httpd_register_uri_handler(server, &save);
    httpd_register_uri_handler(server, &reset);
    httpd_register_uri_handler(server, &blackbox);
    httpd_register_uri_handler(server, &blackbox_clr);

    httpd_uri_t recal = {
        .uri = "/calibrate", .method = HTTP_POST, .handler = calibrate_handler};
    httpd_register_uri_handler(server, &recal);
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
