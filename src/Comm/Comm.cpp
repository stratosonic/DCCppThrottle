#include "Comm.h"
#include "Arduino.h"
#include "Config.h"

#if defined ESP32
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#endif

Comm::Comm(String ipAddress, String port) {
    #if defined ESP32
    this->http = new HTTPClient();
    this->http->begin("http://" + ipAddress + ":" + port);
    //this->http->begin("http://192.168.1.120:2560/");
    this->http->setReuse(true);
    #endif
}

String Comm::send(String packet) {

    Serial.print("sending: ");
    Serial.println(packet);

    String response = "<X>";

    #if defined ESP32

    if(WiFi.status()== WL_CONNECTED) {

        http->addHeader("Content-Type", "text/plain");

        this->http->setTimeout(1000);
        int httpResponseCode = this->http->POST(packet);
        if(httpResponseCode > 0) {
            response = this->http->getString();
            Serial.println(response);
        } else {
            Serial.print("Error on sending POST: ");
            Serial.println(httpResponseCode);
        }
        this->http->end();
    } else {
        Serial.println("Error in WiFi connection");
    }
    return response;

    #elif defined ARDUINO_AVR_MEGA2560
    return "";
    #endif

}

String Comm::sendAndWaitForResponse(String packet) {
    return "<0>";
}
