#ifndef WEBSERVER_H
#define WEBSERVER_H

#include <stdbool.h>

// Initialize Wi-Fi in AP mode and start the HTTP server
void webserver_init(void);

// Set target angles for live display (called from control loop)
void webserver_set_targets(float roll, float pitch);

#endif // WEBSERVER_H
