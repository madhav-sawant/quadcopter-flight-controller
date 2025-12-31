#ifndef WEBSERVER_H
#define WEBSERVER_H

#include <stdbool.h>

// Initialize Wi-Fi in AP mode and start the HTTP server
void webserver_init(void);

// Set target rates for live display (called from control loop)
void webserver_set_rate_targets(float roll_rate, float pitch_rate);

#endif // WEBSERVER_H
