
#include "Arduino.h"

#ifndef CONFIG_H
#define CONFIG_H

#define COMM_SERIAL 0
#define COMM_ETHERNET 1

#define COMM_TYPE COMM_SERIAL

///////////////////////////////////////////////

#define ROTARY_ENCODER_BUTTON_PIN 17 // labeled TX2
#define FOOTER_LEFT_PIN 25
#define FOOTER_MIDDLE_PIN 26
#define FOOTER_RIGHT_PIN 27
#define TRACK_POWER_PIN 33
#define MENU_PIN 32

#define ROTARY_ENCODER_A_PIN 13
#define ROTARY_ENCODER_B_PIN 14

#define SPD_INDICATOR_DIR_PIN 21
#define SPD_INDICATOR_STEP_PIN 22

// (on ESP32, SCK #18, MISO #19, MOSI #23)
#define _cs 15   // goes to TFT CS
#define _dc 2    // goes to TFT DC
#define _mosi 23 // goes to TFT MOSI
#define _sclk 18 // goes to TFT SCK/CLK
#define _rst 4   // goes to TFT RESET
#define _miso 19 // goes to TFT MISO

#define TFT_RST_PIN _rst
#define TFT_DC_PIN _dc
#define TFT_CS_PIN _cs

#endif
