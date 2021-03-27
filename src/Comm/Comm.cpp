#include "Comm.h"
#include "Arduino.h"
#include "Config.h"

#include <WiFi.h>
#include <WiFiClient.h>

Comm::Comm(String ipAddress, uint16_t port) {

    Serial.println("Connected in constructor.");

    //vSemaphoreCreateBinary(this->xSemaphore);
    this->xSemaphore = xSemaphoreCreateMutex();
    sprintf(this->serverIp, ipAddress.c_str());
    this->port = port;
}

String Comm::send(String packet) {
    String response = "<X>";
    xSemaphoreTake(this->xSemaphore, portMAX_DELAY);
    Serial.print("Got semaphore for send - ");
    if (WiFi.status() == WL_CONNECTED) {
        //Serial.print("connected - ");

        WiFiClient client;

        if (!client.connect(this->serverIp, this->port)) {
            Serial.print("Connection failed.");
            return "<Z>";
        } else {
            Serial.print("connected - ");
        }

        Serial.print("sending: ");
        Serial.print(packet);

        client.print(packet);
    } else {
        Serial.println("Error in WiFi connection");
    }
    Serial.println("");
    xSemaphoreGive(this->xSemaphore);
    return response;
}

String Comm::sendAndWaitForResponse(String packet) {
    return "<0>";
}

String Comm::loop() {
    xSemaphoreTake(this->xSemaphore, portMAX_DELAY);
    Serial.print("Got semaphore for recieve - ");
    WiFiClient client;
    String response = "";
    if (client.connect(this->serverIp, this->port)) {
        Serial.print("connected - ");
        if (client.available() > 0) {
            Serial.print("response available - ");

            //int count = client.read(buffer, MAX_ETH_BUFFER);
            response = client.readString();
            //buffer[count] = '\0'; // terminate the string properly

            //response = client.read();
            // Serial.println("res: " + response);
        }
    }
    Serial.println("");
    xSemaphoreGive(this->xSemaphore);

    return response;
}