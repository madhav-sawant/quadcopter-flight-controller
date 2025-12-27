#ifndef WEBSERVER_H
#define WEBSERVER_H

#include <stdbool.h>

// Initialize Wi-Fi in AP mode and start the HTTP server
void webserver_init(void);

// Check if the web server is running
bool webserver_is_running(void);

#endif // WEBSERVER_H
