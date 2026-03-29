/*
 * http_server.h
 * Description: Header file for the LwIP HTTP Web Server
 */

#ifndef HTTP_SERVER_H
#define HTTP_SERVER_H

/* Includes ------------------------------------------------------------------*/
#include "lwip/api.h"

/* Function Prototypes -------------------------------------------------------*/

/**
 * @brief Initializes the web server, binds to port 80, and enters the listening loop.
 * This should be called from inside the FreeRTOS httpServerTask.
 */
void http_server_init(void);

#endif /* HTTP_SERVER_H */
