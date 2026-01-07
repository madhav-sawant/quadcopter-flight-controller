# Ground Station

ESP8266-based ground station receiver for real-time telemetry display. Receives flight data from the quadcopter via nRF24L01 radio and displays it on a web interface.

## Hardware

| Component | Description |
|-----------|-------------|
| ESP8266 | NodeMCU or similar |
| nRF24L01+ | 2.4GHz radio module |
| Power | 5V via USB |

### Wiring (ESP8266 to nRF24L01)

| nRF24L01 | ESP8266 |
|----------|---------|
| VCC | 3.3V |
| GND | GND |
| CE | D2 (GPIO4) |
| CSN | D8 (GPIO15) |
| SCK | D5 (GPIO14) |
| MOSI | D7 (GPIO13) |
| MISO | D6 (GPIO12) |

## Features

- Receives telemetry at 2.4GHz
- Web interface at 192.168.4.1
- Displays: Roll, Pitch, Yaw, Battery, GPS coordinates
- No app installation needed - works in any browser

## How to Flash

1. Open `ground_station.ino` in Arduino IDE
2. Install ESP8266 board (add `http://arduino.esp8266.com/stable/package_esp8266com_index.json` to board manager)
3. Install RF24 library from Library Manager
4. Select board: NodeMCU 1.0
5. Upload

## Usage

1. Power on the ground station
2. Connect to WiFi: `DroneGCS` (password: `12345678`)
3. Open browser: `http://192.168.4.1`
4. Flight data updates in real-time
