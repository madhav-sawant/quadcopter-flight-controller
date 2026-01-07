/*
 * Ground Station Receiver
 * ESP8266 + nRF24L01
 *
 * Receives telemetry from quadcopter and displays on web interface
 */

#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <RF24.h>
#include <SPI.h>

// nRF24L01 pins
#define CE_PIN 4   // D2
#define CSN_PIN 15 // D8

// WiFi AP settings
const char *ssid = "DroneGCS";
const char *password = "12345678";

// Radio setup
RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "DRONE";

// Web server
ESP8266WebServer server(80);

// Telemetry data structure - must match drone side
struct TelemetryPacket {
  float roll;
  float pitch;
  float yaw;
  float battery_v;
  float latitude;
  float longitude;
  float altitude;
  uint8_t armed;
  uint8_t gps_fix;
} telemetry;

unsigned long last_packet_time = 0;
bool connected = false;

// HTML page
const char *html_page = R"(
<!DOCTYPE html>
<html>
<head>
  <title>Ground Station</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial; background: #1a1a2e; color: #eee; padding: 20px; }
    .container { max-width: 400px; margin: 0 auto; }
    h1 { text-align: center; color: #00d4ff; }
    .card { background: #16213e; padding: 15px; margin: 10px 0; border-radius: 8px; }
    .label { color: #888; font-size: 12px; }
    .value { font-size: 24px; font-weight: bold; }
    .status { text-align: center; padding: 10px; border-radius: 5px; }
    .connected { background: #00aa00; }
    .disconnected { background: #aa0000; }
    .armed { background: #ff4444; }
    .disarmed { background: #44aa44; }
    .row { display: flex; justify-content: space-between; }
    .col { flex: 1; text-align: center; }
  </style>
</head>
<body>
  <div class="container">
    <h1>Ground Station</h1>
    <div id="status" class="status disconnected">Waiting for signal...</div>
    <div id="arm-status" class="status disarmed" style="margin-top:5px">DISARMED</div>
    
    <div class="card">
      <div class="row">
        <div class="col">
          <div class="label">ROLL</div>
          <div class="value" id="roll">--</div>
        </div>
        <div class="col">
          <div class="label">PITCH</div>
          <div class="value" id="pitch">--</div>
        </div>
        <div class="col">
          <div class="label">YAW</div>
          <div class="value" id="yaw">--</div>
        </div>
      </div>
    </div>
    
    <div class="card">
      <div class="label">BATTERY</div>
      <div class="value" id="battery">--</div>
    </div>
    
    <div class="card">
      <div class="label">GPS</div>
      <div class="value" id="gps" style="font-size:16px">--</div>
      <div class="label" style="margin-top:5px">ALTITUDE</div>
      <div class="value" id="alt">--</div>
    </div>
  </div>
  
  <script>
    function update() {
      fetch('/data')
        .then(r => r.json())
        .then(d => {
          document.getElementById('roll').innerText = d.roll.toFixed(1) + '°';
          document.getElementById('pitch').innerText = d.pitch.toFixed(1) + '°';
          document.getElementById('yaw').innerText = d.yaw.toFixed(1) + '°';
          document.getElementById('battery').innerText = d.battery.toFixed(1) + 'V';
          document.getElementById('alt').innerText = d.alt.toFixed(1) + 'm';
          
          if (d.gps_fix) {
            document.getElementById('gps').innerText = d.lat.toFixed(6) + ', ' + d.lon.toFixed(6);
          } else {
            document.getElementById('gps').innerText = 'No Fix';
          }
          
          var status = document.getElementById('status');
          if (d.connected) {
            status.innerText = 'CONNECTED';
            status.className = 'status connected';
          } else {
            status.innerText = 'NO SIGNAL';
            status.className = 'status disconnected';
          }
          
          var arm = document.getElementById('arm-status');
          if (d.armed) {
            arm.innerText = 'ARMED';
            arm.className = 'status armed';
          } else {
            arm.innerText = 'DISARMED';
            arm.className = 'status disarmed';
          }
        });
    }
    setInterval(update, 200);
    update();
  </script>
</body>
</html>
)";

void setup() {
  Serial.begin(115200);
  Serial.println("\nGround Station Starting...");

  // Initialize radio
  if (!radio.begin()) {
    Serial.println("Radio init failed!");
  } else {
    Serial.println("Radio OK");
  }

  radio.openReadingPipe(1, address);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.startListening();

  // Start WiFi AP
  WiFi.softAP(ssid, password);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  // Setup web routes
  server.on("/", []() { server.send(200, "text/html", html_page); });

  server.on("/data", []() {
    String json = "{";
    json += "\"roll\":" + String(telemetry.roll, 2) + ",";
    json += "\"pitch\":" + String(telemetry.pitch, 2) + ",";
    json += "\"yaw\":" + String(telemetry.yaw, 2) + ",";
    json += "\"battery\":" + String(telemetry.battery_v, 2) + ",";
    json += "\"lat\":" + String(telemetry.latitude, 6) + ",";
    json += "\"lon\":" + String(telemetry.longitude, 6) + ",";
    json += "\"alt\":" + String(telemetry.altitude, 1) + ",";
    json += "\"armed\":" + String(telemetry.armed) + ",";
    json += "\"gps_fix\":" + String(telemetry.gps_fix) + ",";
    json += "\"connected\":" + String(connected ? "true" : "false");
    json += "}";
    server.send(200, "application/json", json);
  });

  server.begin();
  Serial.println("Web server started");
}

void loop() {
  // Check for radio data
  if (radio.available()) {
    radio.read(&telemetry, sizeof(telemetry));
    last_packet_time = millis();
    connected = true;
  }

  // Timeout check
  if (millis() - last_packet_time > 1000) {
    connected = false;
  }

  // Handle web requests
  server.handleClient();
}
