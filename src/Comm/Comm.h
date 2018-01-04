#include "Arduino.h"
#include <HTTPClient.h>

#ifndef COMM_H
#define COMM_H

class Comm {
private:
HTTPClient *http;
public:
Comm();
String sendAndWaitForResponse(String packet);
String send(String packet);
};

#endif
