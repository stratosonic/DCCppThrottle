#include "Arduino.h"
#include <WiFiClient.h>

#ifndef COMM_H
#define COMM_H

//#define MAX_ETH_BUFFER 512

class Comm {
  private:
    WiFiClient wclient;
    xSemaphoreHandle xSemaphore;
    char serverIp[17];
    uint16_t port;

  public:
    Comm(String ipAddress, uint16_t port);
    String sendAndWaitForResponse(String packet);
    String send(String packet);
    String loop();
};

#endif
