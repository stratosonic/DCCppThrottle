#include <Arduino.h>
#include <HTTPClient.h>
#include <LinkedList.h>
#include <WiFi.h>

#include "Accessory/Accessory.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include "AiEsp32RotaryEncoder.h"
#include "Comm/Comm.h"
#include "Config.h"
#include "SPI.h"
#include "Sensor/Sensor.h"
#include "SpeedIndicator/SpeedIndicator.h"
#include "Train/Train.h"
#include "Turnout/Turnout.h"
#include <RunningMedian.h>

// clang-format off

// Set to 1 for extra debug messages to serial out
#define DEBUG 1

hw_timer_t * timer = NULL;

// Use hardware SPI
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS_PIN, TFT_DC_PIN, TFT_RST_PIN);

//tft.color565(0,0,0);
#define ILI9341_DARKRED    0xB000 /* 22, 0, 0 */
#define ILI9341_DARKERGREY 0x38E7 /* 00111000 11100111 */

#define SCREEN_WIDTH   320
#define SCREEN_HEIGHT  240

#define FONT_SIZE_1_WIDTH   6
#define FONT_SIZE_1_HEIGHT  8
#define FONT_SIZE_2_WIDTH  12
#define FONT_SIZE_2_HEIGHT 16

#define BATTERY_WIDTH  28
#define BATTERY_HEIGHT 10

#define BATTERY_FULL          24
#define BATTERY_THREE_QUARTER 18
#define BATTERY_HALF          12
#define BATTERY_ONE_QUARTER    6
#define BATTERY_EMPTY          1

#define MAIN_VIEW_TOP_Y 38

#define FOOTER_BUTTON_COLOR ILI9341_RED
#define FOOTER_BUTTON_HIGHLIGHT_COLOR ILI9341_DARKRED
#define FOOTER_BUTTON_LEFT   0
#define FOOTER_BUTTON_MIDDLE 1
#define FOOTER_BUTTON_RIGHT  2

char footerButtonsLabel[3][9] = {"", "", ""};

#define BUTTON_DEBOUNCE_MS       10 // how many ms to debounce

#define ENCODER_BUTTON_PIN       ROTARY_ENCODER_BUTTON_PIN
#define FOOTER_BUTTON_LEFT_PIN   FOOTER_LEFT_PIN
#define FOOTER_BUTTON_MIDDLE_PIN FOOTER_MIDDLE_PIN
#define FOOTER_BUTTON_RIGHT_PIN  FOOTER_RIGHT_PIN
#define TRACK_POWER_BUTTON_PIN   TRACK_POWER_PIN
#define MENU_BUTTON_PIN          MENU_PIN

byte buttons[] = { ENCODER_BUTTON_PIN,
                   FOOTER_BUTTON_LEFT_PIN,
                   FOOTER_BUTTON_MIDDLE_PIN,
                   FOOTER_BUTTON_RIGHT_PIN,
                   TRACK_POWER_BUTTON_PIN,
                   MENU_BUTTON_PIN};

#define NUMBER_OF_BUTTONS sizeof(buttons)

// we will track if a button is currently pressed, just pressed, or just released
volatile byte pressedButton[NUMBER_OF_BUTTONS], justPressedButton[NUMBER_OF_BUTTONS], justReleasedButton[NUMBER_OF_BUTTONS];

#define ENCODER_BUTTON_JUST_PRESSED       justReleasedButton[0]
#define FOOTER_BUTTON_LEFT_JUST_PRESSED   justReleasedButton[1]
#define FOOTER_BUTTON_MIDDLE_JUST_PRESSED justReleasedButton[2]
#define FOOTER_BUTTON_RIGHT_JUST_PRESSED  justReleasedButton[3]
#define TRACK_POWER_BUTTON_JUST_PRESSED   justReleasedButton[4]
#define MENU_BUTTON_JUST_PRESSED          justReleasedButton[5]

#define ROTARY_ENCODER_STEPS 4
#define ROTARY_ENCODER_VCC_PIN -1

AiEsp32RotaryEncoder rotaryEncoder = 
     AiEsp32RotaryEncoder(
        ROTARY_ENCODER_A_PIN, 
        ROTARY_ENCODER_B_PIN, 
        ROTARY_ENCODER_BUTTON_PIN, 
        ROTARY_ENCODER_VCC_PIN, 
        ROTARY_ENCODER_STEPS);

#define ROT_ENC_TRAIN_SPEED_MIN -126
#define ROT_ENC_TRAIN_SPEED_MAX 126
#define ROT_ENC_TRAIN_ACCEL 300
#define ROT_ENC_MENU_MIN -1
#define ROT_ENC_MENU_MAX 5
#define ROT_ENC_MENU_ACCEL 0

//Comm *comm = new Comm("192.168.1.120", "2560");
Comm *comm;

#define MAX_COMMAND_LENGTH 255
char commandString[MAX_COMMAND_LENGTH];

#define TRACK_POWER_ON    1
#define TRACK_POWER_OFF   0
bool trackPower = TRACK_POWER_OFF;

// counter for how ofter to check track current
RunningMedian trackCurrentSamples = RunningMedian(5);
#define TRACK_CURRENT_REQUEST_FREQUENCY 100  // 100 = once per second
int trackCurrentCounter = 0;
float trackCurrent = 0.0;

// Previous speed value for display checking to avoid flicker
char previousSpeed[3];

byte menuViewSelection = 0;
byte previousMenuViewSelection = 0;
char menuText[5][15] = {"Control Trains", "Turnouts", "Accessories", "Sensors", "Preferences"};
#define NUMBER_OF_MENU_ITEMS 5

// Maintain the order of the menu to match the menuText array
enum DisplayState { TRAIN_CONTROL, TURNOUT, ACCESSORIES, SENSORS, PREFERENCES, START, MENU };
DisplayState displayState = START;
DisplayState previousDisplayState = displayState;

enum EncoderState { NIL, NEXT, PREVIOUS };
EncoderState encoderState = NIL;

byte previouslySelectedFunctionNumber = 28;

byte currentTrainDirection = -1;

LinkedList<Train *> trainList = LinkedList<Train *>();
Train *currentTrain;
byte currentTrainIndex = 0;

LinkedList<Turnout *> turnoutList = LinkedList<Turnout *>();
int8_t selectedTurnout = 0;
int8_t previousTurnout = 0;
#define TURNOUT_ROWS 3
#define TURNOUT_COLUMNS 2

LinkedList<Accessory *> accessoryList = LinkedList<Accessory *>();
int8_t selectedAccessory = 0;
int8_t previousAccessory = 0;
#define ACCESSORY_ROWS 2
#define ACCESSORY_COLUMNS 2

LinkedList<Sensor *> sensorList = LinkedList<Sensor *>();
int8_t selectedSensor = 0;
int8_t previousSensor = 0;
#define SENSOR_ROWS 2
#define SENSOR_COLUMNS 3

char spinnerArray[] = {'-', '\\', '|', '/', '-', '\\', '|', '/'};
byte spinnerArrayPointer = 0;

// clang-format on

//////
void onTimer1();

void sendTask(void *parameter);
void receiveTask(void *parameter);

void drawSpinner();
void drawStartView();
void drawStartViewContinue();
void parseCommand(char *);
void updateEncoder();
void buttonScan();
void clearPreviousMenuSelection();
void clearView();
void clearButtonStates();
void clearEncoderState();
void clearFunctionRows();
void drawTrainView();
void drawHeader();
void drawMenuView();
void drawMenuItem();
void drawMenuItems();
void drawAccessoriesView();
void drawSensorsView();
void drawPreferencesView();
void drawCurrentMenuSelection();
void drawFooterButtonsLabel(char *, char *, char *);
template <typename T>
void drawBlockView(uint8_t numberOfRows, uint8_t numberOfColumns, LinkedList<T> *list, byte selectedIndex, byte lastIndex, bool drawAll);
void drawTrainRow(Train *);
void drawTrainNumber(int);
void drawTrainDirection(byte);
void drawTrainFunctions(Train *);
void drawTrainSpeed(int, byte);
void updateTrainFunctions(Train *);
void drawTurnoutView();
void toggleTrackPower();
void sendTrackPowerCommand();
void sendCurrentRequestCommand();
void sendTrainCommand(Train *);
void drawTrackPower();
void drawTrackCurrent();
void drawWifiSymbol(int, int);
void drawBattery(int, int, byte);
void parseTCommand(char *command);
void parseHCommand(char *command);
void enableTimerInterrupt();
void disableTimerInterrupt();
void readResponse(char *response);
void resetRotaryEncoderValues(long val, long min, long max, unsigned long accel, bool circleValues);
void send(char *packet);
void resetGauge(void *parameter);
void moveGauge(void *parameter);
/////

enum WifiStrength { EXCELLENT,
                    FAIR,
                    POOR,
                    NONE };
WifiStrength wifiStrength = NONE;
WifiStrength prevWifiStrength = NONE;
int wifiSymbolCounter = 0;
int8_t rssi = 0;
HTTPClient *http;
xSemaphoreHandle sendReceiveSemaphore;
WiFiClient wclient;
IPAddress serverip(192, 168, 4, 1);

#define MAX_ETH_BUFFER 2048
char recieveBuffer[MAX_ETH_BUFFER];

LinkedList<char *> outgoingCommands = LinkedList<char *>();

unsigned long wifiClientTimeout = 0;

xSemaphoreHandle speedIndicatorSemaphore;
SpeedIndicator *speedIndicator;
int8_t speedIndicatorValue = 0;

void rotary_onButtonClick() {}

void setup() {
    Serial.begin(115200);
    if (DEBUG == 1) {
        Serial.println(F("DCC++ Throttle"));
    }

    tft.begin();
    tft.setRotation(3);
    tft.fillScreen(ILI9341_BLACK);

    drawStartView();

    // First disconnect so reconnection will work
    WiFi.disconnect(false);

    // Initialize Speed Indicator
    speedIndicatorSemaphore = xSemaphoreCreateMutex();
    speedIndicator = new SpeedIndicator(SPD_INDICATOR_STEP_PIN, SPD_INDICATOR_DIR_PIN);
    xTaskCreate(resetGauge, "ResetGauge", 1024, NULL, 1, NULL);

    //Delay approx 4 seconds before calling the WiFi.begin
    for (byte i = 0; i < 53; i++) {
        drawSpinner();
        delay(75);
    }

    WiFi.begin(SSID_NAME, SSID_PASSWORD);

    while (WiFi.status() != WL_CONNECTED) { //Check for the connection
        drawSpinner();
        delay(75);
        Serial.println(F("Connecting to WiFi.."));
    }
    Serial.println(F("Connected to WiFi network"));

    //we must initialize rotary encoder
    rotaryEncoder.begin();
    rotaryEncoder.setup(
        [] { rotaryEncoder.readEncoder_ISR(); },
        [] { rotary_onButtonClick(); });
    resetRotaryEncoderValues(0, ROT_ENC_TRAIN_SPEED_MIN, ROT_ENC_TRAIN_SPEED_MAX, ROT_ENC_TRAIN_ACCEL, false);

    // Set ports for buttons to inputs with internal pullup
    for (byte i = 0; i < NUMBER_OF_BUTTONS; i++) {
        pinMode(buttons[i], INPUT_PULLUP);
    }

    // ----------  Add Trains  ---------- //
    Train *train1 = new Train(1, 8671, MAX_NUMBER_OF_TRAIN_FUNCTIONS);
    trainList.add(train1);
    Train *train2 = new Train(1, 1234, MAX_NUMBER_OF_TRAIN_FUNCTIONS);
    trainList.add(train2);
    Train *train3 = new Train(1, 10035, 8);
    trainList.add(train3);
    currentTrain = trainList.get(currentTrainIndex);

    // ----------  Add Turnouts  ---------- //
    Turnout *turnout1 = new Turnout(1, (char *)"Turnout 1");
    turnoutList.add(turnout1);
    Turnout *turnout2 = new Turnout(2, (char *)"Yard 1");
    turnoutList.add(turnout2);
    Turnout *turnout3 = new Turnout(3, (char *)"Yard 2");
    turnoutList.add(turnout3);
    Turnout *turnout4 = new Turnout(4, (char *)"Warehouse");
    turnoutList.add(turnout4);
    Turnout *turnout5 = new Turnout(5, (char *)"Extra A");
    turnoutList.add(turnout5);
    Turnout *turnout6 = new Turnout(6, (char *)"Extra B");
    turnoutList.add(turnout6);

    // ----------  Add Accessories  ---------- //
    accessoryList.add(new Accessory((char *)"A1", 1, 1));
    accessoryList.add(new Accessory((char *)"Light", 1, 2));
    accessoryList.add(new Accessory((char *)"Swing", 1, 3));
    accessoryList.add(new Accessory((char *)"Blah", 1, 4));

    // ----------  Add Sensors  ---------- //
    sensorList.add(new Sensor((char *)"Snsr 1"));
    sensorList.add(new Sensor((char *)"Snsr 2"));
    sensorList.add(new Sensor((char *)"S 3"));
    sensorList.add(new Sensor((char *)"S 4"));
    sensorList.add(new Sensor((char *)"S 5"));
    sensorList.add(new Sensor((char *)"S 6"));

    if (DEBUG == 1) {
        Serial.println(F("Done with setup"));
    }

    // https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/
    // TIMER 1 for interrupt frequency of 100 Hz:
    // Clock frequency: 80Mhz
    // 80,000,000 / 8000 / 100 = 100 Hz

    timer = timerBegin(0, 8000, true); // 80000000 / 8000 (must be <65536) = 10000 Hz
    enableTimerInterrupt();

    // hide spinner
    tft.fillRect(152, 130, FONT_SIZE_2_WIDTH, FONT_SIZE_2_HEIGHT, ILI9341_BLACK);

    drawStartViewContinue();

    //comm = new Comm("192.168.1.120", 2560);

    sendReceiveSemaphore = xSemaphoreCreateMutex();

    wclient.connect(serverip, 2560);

    //xBinarySemaphore = xSemaphoreCreateBinary();

    xTaskCreatePinnedToCore(
        sendTask,   /* Function to implement the task */
        "SendTask", /* Name of the task */
        10000,      /* Stack size in words */
        NULL,       /* Task input parameter */
        10,         /* Priority of the task */
        NULL,       /* Task handle. */
        0);         /* Core where the task should run */

    xTaskCreatePinnedToCore(
        receiveTask,   /* Function to implement the task */
        "ReceiveTask", /* Name of the task */
        10000,         /* Stack size in words */
        NULL,          /* Task input parameter */
        10,            /* Priority of the task */
        NULL,          /* Task handle. */
        1);            /* Core where the task should run */

    // Send status request
    send((char *)"<s>");

    // Request all defined turnouts
    send((char *)"<T>");

    // Request all defined Sensors
    send((char *)"<S>");

    // Request all defined output pins
    send((char *)"<Z>");

    // Status of all sensors ???
    send((char *)"<Q>");
}

void enableTimerInterrupt() {
    timerAttachInterrupt(timer, &onTimer1, true);
    timerAlarmWrite(timer, 100, true); // 10000 / 100 = 100Hz
    timerAlarmEnable(timer);
}

void disableTimerInterrupt() {
    timerDetachInterrupt(timer);
}

void drawSpinner() {
    tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
    tft.setCursor(152, 130);
    tft.print(spinnerArray[spinnerArrayPointer]);
    spinnerArrayPointer++;
    if (spinnerArrayPointer >= sizeof(spinnerArray)) {
        spinnerArrayPointer = 0;
    }
}

void readResponse(char *response) {
    int bufferLength = strlen(response) + 1;
    char buffer[bufferLength] = "";
    strcpy(buffer, response);

    for (int i = 0; i < bufferLength; i++) {
        int c = buffer[i]; //Serial.read();

        if (c == '<') { // start of new command, reset the string
            commandString[0] = '\0';
        } else if (c == '>') { // end of new command
            parseCommand(commandString);
        } else if (strlen(commandString) < MAX_COMMAND_LENGTH) { // if comandString still has space, append character just read from serial line
            sprintf(commandString, "%s%c", commandString, c);    // otherwise, character is ignored (but continue to look for '<' or '>')
        }
    }
}

void sendTask(void *parameter) {
    while (true) {

        if (outgoingCommands.size() > 0) {
            xSemaphoreTake(sendReceiveSemaphore, portMAX_DELAY);
            //Serial.print("outgoingCommands size: ");
            //Serial.println(outgoingCommands.size());
            char *packet = outgoingCommands.shift();
            xSemaphoreGive(sendReceiveSemaphore);
            // Serial.print("outgoing packet: ");
            // Serial.println(packet);
            // unsigned long start = micros();
            wclient.write(packet);
            // unsigned long end = micros();
            // unsigned long time = end - start;
            // Serial.print("send time: ");
            // Serial.println(time);
        }
        //https://esp32.com/viewtopic.php?t=11371
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void receiveTask(void *parameter) {
    while (true) {
        if (!wclient.connected()) {
            Serial.println("reconnecting to server");
            while (!wclient.connect(serverip, 2560)) {
                Serial.print(".");
            }
        }

        if (wclient.available() > 0) {
            //wclient.setTimeout(10);
            //unsigned long start = micros();
            //wclient.setNoDelay(true);
            //xSemaphoreTake(sendReceiveSemaphore, portMAX_DELAY);
            int readBytes = wclient.readBytesUntil('>', recieveBuffer, MAX_ETH_BUFFER - 2);
            //xSemaphoreGive(sendReceiveSemaphore);
            //wclient.flush();
            //unsigned long end = micros();
            recieveBuffer[readBytes] = '>';
            recieveBuffer[readBytes + 1] = '\0';
            readResponse(recieveBuffer);
            //Serial.println(recieveBuffer);
            //wclient.setTimeout(wifiClientTimeout);
            //unsigned long time = end - start;
            //Serial.print("recv time: ");
            //Serial.print(time);
            //Serial.print(", readBytes: ");
            //Serial.println(readBytes);
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void send(char *packet) {
    xSemaphoreTake(sendReceiveSemaphore, portMAX_DELAY);
    //Serial.print("Sending: ");
    //Serial.println(packet);
    outgoingCommands.add(packet);
    xSemaphoreGive(sendReceiveSemaphore);
}

void resetRotaryEncoderValues(long val, long min, long max, unsigned long accel, bool circleValues) {
    rotaryEncoder.setEncoderValue(val);
    rotaryEncoder.setBoundaries(min, max, circleValues);
    rotaryEncoder.setAcceleration(accel);
}

void loop() {

    if (previousDisplayState != displayState) {
        if (DEBUG == 1) {
            Serial.print(F("displayState "));
            Serial.println(displayState);
        }
        previousDisplayState = displayState;
    }

    // Every loop
    if (displayState != START) {
        if (TRACK_POWER_BUTTON_JUST_PRESSED) {
            TRACK_POWER_BUTTON_JUST_PRESSED = 0;
            toggleTrackPower();
            sendTrackPowerCommand();
            drawTrackPower();
            if (displayState == TRAIN_CONTROL) {
                drawTrainSpeed(currentTrain->getSpeed(), currentTrain->direction);
            }
        }
    }

    // 2Hz (2 times per second)
    if (trackCurrentCounter >= 50) {
        Serial.print("free memory: ");
        Serial.println(xPortGetFreeHeapSize());
        disableTimerInterrupt();
        sendCurrentRequestCommand();
        trackCurrentCounter = 0;
        enableTimerInterrupt();
        drawTrackCurrent();
    }

    // 1Hz (1 time per second)
    if (wifiSymbolCounter >= 100) {
        if (displayState != START) {
            drawWifiSymbol(260, 4);
        }
        wifiSymbolCounter = 0;
    }

    // Main state machine check
    switch (displayState) {

        // ---------------------------------------------- //
        case START:
            if (ENCODER_BUTTON_JUST_PRESSED ||
                FOOTER_BUTTON_LEFT_JUST_PRESSED ||
                FOOTER_BUTTON_MIDDLE_JUST_PRESSED ||
                FOOTER_BUTTON_RIGHT_JUST_PRESSED ||
                TRACK_POWER_BUTTON_JUST_PRESSED ||
                MENU_BUTTON_JUST_PRESSED) {
                clearButtonStates();
                clearView();
                // Draw header background
                drawHeader();
                drawTrainView();

                displayState = TRAIN_CONTROL;
                resetRotaryEncoderValues(currentTrain->getSpeed(), ROT_ENC_TRAIN_SPEED_MIN,
                                         ROT_ENC_TRAIN_SPEED_MAX, ROT_ENC_TRAIN_ACCEL, false);
            }
            break;

            // --------------------- TRAIN_CONTROL ------------------------- //

        case TRAIN_CONTROL:
            if (FOOTER_BUTTON_LEFT_JUST_PRESSED) {
                clearButtonStates();
                currentTrainIndex++;
                if (currentTrainIndex >= trainList.size()) {
                    currentTrainIndex = 0;
                }
                currentTrain = trainList.get(currentTrainIndex);
                drawTrainView();
            } else if (FOOTER_BUTTON_MIDDLE_JUST_PRESSED) {
                clearButtonStates();
                currentTrain->incrementActiveFunction();
                updateTrainFunctions(currentTrain);
            } else if (FOOTER_BUTTON_RIGHT_JUST_PRESSED) {
                clearButtonStates();
                char functionCommand[18];
                currentTrain->getFunctionCommand(functionCommand);
                send(functionCommand);
                updateTrainFunctions(currentTrain);
            } else if (MENU_BUTTON_JUST_PRESSED) {
                clearButtonStates();
                clearView();
                displayState = MENU;
                resetRotaryEncoderValues(0, ROT_ENC_MENU_MIN, ROT_ENC_MENU_MAX, ROT_ENC_MENU_ACCEL, true);
                drawMenuView();
            }

            if (ENCODER_BUTTON_JUST_PRESSED) {
                clearButtonStates();
                rotaryEncoder.setEncoderValue(0);
                if (currentTrain->getSpeed() > 0 || currentTrain->getSpeed() < 0) {
                    currentTrain->setSpeed(0);
                    drawTrainSpeed(currentTrain->getSpeed(), currentTrain->direction);
                } else if (currentTrain->getSpeed() == 0) {
                    currentTrain->direction = !currentTrain->direction;
                }
                sendTrainCommand(currentTrain);
                drawTrainDirection(currentTrain->direction);
            }

            {
                if (rotaryEncoder.encoderChanged()) {
                    long currentTrainSpeed = rotaryEncoder.readEncoder();
                    currentTrain->setSpeed(currentTrainSpeed);
                    drawTrainSpeed(currentTrain->getSpeed(), currentTrain->direction);
                    if (currentTrainDirection != currentTrain->direction) {
                        drawTrainDirection(currentTrain->direction);
                        currentTrainDirection = currentTrain->direction;
                    }
                    sendTrainCommand(currentTrain);
                }
            }

            break;

        // ---------------------- MENU ------------------------ //
        case MENU:
            if (rotaryEncoder.encoderChanged()) {
                long menuSelection = rotaryEncoder.readEncoder();
                if (menuSelection < 0) {
                    menuViewSelection = NUMBER_OF_MENU_ITEMS - 1;
                    rotaryEncoder.setEncoderValue(NUMBER_OF_MENU_ITEMS - 1);
                } else if (menuSelection > NUMBER_OF_MENU_ITEMS - 1) {
                    menuViewSelection = 0;
                    rotaryEncoder.setEncoderValue(0);
                } else {
                    menuViewSelection = menuSelection;
                }
                drawMenuItem();
            }

            if (ENCODER_BUTTON_JUST_PRESSED) {
                clearButtonStates();
                clearEncoderState();
                clearView();
                if (menuViewSelection == TRAIN_CONTROL) {
                    displayState = TRAIN_CONTROL;
                    resetRotaryEncoderValues(currentTrain->getSpeed(), ROT_ENC_TRAIN_SPEED_MIN,
                                             ROT_ENC_TRAIN_SPEED_MAX, ROT_ENC_TRAIN_ACCEL, false);
                    drawTrainView();
                } else if (menuViewSelection == TURNOUT) {
                    displayState = TURNOUT;
                    resetRotaryEncoderValues(0, 0, turnoutList.size(), 0, true);
                    drawTurnoutView();
                } else if (menuViewSelection == ACCESSORIES) {
                    displayState = ACCESSORIES;
                    resetRotaryEncoderValues(0, 0, accessoryList.size(), 0, true);
                    drawAccessoriesView();
                } else if (menuViewSelection == SENSORS) {
                    displayState = SENSORS;
                    resetRotaryEncoderValues(0, 0, sensorList.size(), 0, true);
                    drawSensorsView();
                } else if (menuViewSelection == PREFERENCES) {
                    displayState = PREFERENCES;
                    resetRotaryEncoderValues(0, 0, 5, 0, true);
                    drawPreferencesView();
                }
            }

            break;

        // ---------------------- TURNOUT ------------------------ //
        case TURNOUT:
            if (MENU_BUTTON_JUST_PRESSED) {
                clearButtonStates();
                clearView();
                displayState = MENU;
                resetRotaryEncoderValues(1, ROT_ENC_MENU_MIN, ROT_ENC_MENU_MAX, ROT_ENC_MENU_ACCEL, true);
                drawMenuView();
            } else {
                {
                    if (rotaryEncoder.encoderChanged()) {
                        selectedTurnout = rotaryEncoder.readEncoder();
                        Serial.println(selectedTurnout);
                        drawBlockView(TURNOUT_ROWS, TURNOUT_COLUMNS, &turnoutList, selectedTurnout, previousTurnout, false);
                        previousTurnout = selectedTurnout;
                    }
                }

                if (ENCODER_BUTTON_JUST_PRESSED) {
                    clearButtonStates();
                    Turnout *turnout = turnoutList.get(selectedTurnout);
                    turnout->toggleActive();
                    drawBlockView(TURNOUT_ROWS, TURNOUT_COLUMNS, &turnoutList, selectedTurnout, previousTurnout, false);
                }
            }
            break;

        // ---------------------- ACCESSORIES ------------------------ //
        case ACCESSORIES:
            if (MENU_BUTTON_JUST_PRESSED) {
                clearButtonStates();
                clearView();
                displayState = MENU;
                resetRotaryEncoderValues(2, ROT_ENC_MENU_MIN, ROT_ENC_MENU_MAX, ROT_ENC_MENU_ACCEL, true);
                drawMenuView();
            } else {
                {
                    if (rotaryEncoder.encoderChanged()) {
                        selectedAccessory = rotaryEncoder.readEncoder();
                        drawBlockView(ACCESSORY_ROWS, ACCESSORY_COLUMNS, &accessoryList, selectedAccessory, previousAccessory, false);
                        previousAccessory = selectedAccessory;
                    }
                }

                if (ENCODER_BUTTON_JUST_PRESSED) {
                    clearButtonStates();
                    Accessory *accessory = accessoryList.get(selectedAccessory);
                    accessory->toggleActive();
                    drawBlockView(ACCESSORY_ROWS, ACCESSORY_COLUMNS, &accessoryList, selectedAccessory, previousAccessory, false);
                }
            }
            break;

        // ----------------------- SENSORS ----------------------- //
        case SENSORS:
            if (MENU_BUTTON_JUST_PRESSED) {
                clearButtonStates();
                clearView();
                displayState = MENU;
                resetRotaryEncoderValues(3, ROT_ENC_MENU_MIN, ROT_ENC_MENU_MAX, ROT_ENC_MENU_ACCEL, true);
                drawMenuView();
            } else {
                {
                    if (rotaryEncoder.encoderChanged()) {
                        selectedSensor = rotaryEncoder.readEncoder();
                        drawBlockView(SENSOR_ROWS, SENSOR_COLUMNS, &sensorList, selectedSensor, previousSensor, false);
                        previousSensor = selectedSensor;
                    }
                }

                if (ENCODER_BUTTON_JUST_PRESSED) {
                    clearButtonStates();
                    Sensor *sensor = sensorList.get(selectedSensor);
                    sensor->toggleActive();
                    drawBlockView(SENSOR_ROWS, SENSOR_COLUMNS, &sensorList, selectedSensor, previousSensor, false);
                }
            }
            break;

        // ----------------------- PREFERENCES ----------------------- //
        case PREFERENCES:
            if (MENU_BUTTON_JUST_PRESSED) {
                clearButtonStates();
                clearView();
                displayState = MENU;
                resetRotaryEncoderValues(4, ROT_ENC_MENU_MIN, ROT_ENC_MENU_MAX, ROT_ENC_MENU_ACCEL, true);
                drawMenuView();
            }
            break;

        default:
            break;
    }
}

void onTimer1() {
    buttonScan();
    trackCurrentCounter++;
    wifiSymbolCounter++;
}

void clearButtonStates() {
    for (byte i = 0; i < NUMBER_OF_BUTTONS; i++) {
        justReleasedButton[i] = 0;
    }
}

void clearEncoderState() {
    encoderState = NIL;
}

void drawHeader() {
    tft.drawFastHLine(0, 25, 320, ILI9341_DARKGREY);
    tft.setCursor(5, 5);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);
    tft.print(F("TRK PWR:"));
    drawTrackPower();
    tft.setCursor(FONT_SIZE_2_WIDTH * 17, 5);
    tft.setTextColor(ILI9341_WHITE);
    tft.print("A");
    drawTrackCurrent();
    drawBattery(287, 6, BATTERY_FULL);
}

void drawBattery(int x, int y, byte batteryLevel) {
    // Draw battery
    tft.drawRoundRect(x, y, BATTERY_WIDTH, BATTERY_HEIGHT, 2, ILI9341_WHITE);
    tft.fillRect(x + BATTERY_WIDTH, y + 2, 2, BATTERY_HEIGHT - 4, ILI9341_WHITE);

    // Blank out battery level
    tft.fillRect(x + 2, y + 2, BATTERY_WIDTH - 4, BATTERY_HEIGHT - 4, ILI9341_BLACK);

    int batteryColor = ILI9341_GREEN;
    if (batteryLevel == BATTERY_EMPTY) {
        batteryColor = ILI9341_RED;
    }
    // Draw battery level
    tft.fillRect(x + 2, y + 2, batteryLevel, BATTERY_HEIGHT - 4, batteryColor);
}

/**
 * Draws the Wifi Symbol.
 * 
 *    > -50     excellent
 *  -50 to -70  fair
 *  -70 to -80  poor
 *    < -80     no signal
 * 
 * @param x int x position of the symbol
 * @param y int y position of the symbol
 * @return
 * 

 */
void drawWifiSymbol(int x, int y) {

    //Serial.print("RSSI: ");
    //Serial.println(rssi);

    int8_t rssi = WiFi.RSSI();
    if (rssi > -50) {
        wifiStrength = EXCELLENT;
    } else if (rssi > -70) {
        wifiStrength = FAIR;
    } else if (rssi > -80) {
        wifiStrength = POOR;
    } else {
        wifiStrength = NONE;
    }

    if (prevWifiStrength == wifiStrength) {
        return;
    }
    prevWifiStrength = wifiStrength;

    bool connected = true;
    if (WiFi.status() != WL_CONNECTED) {
        connected = false;
    }

    byte pixelSize = 2;
    int color = ILI9341_RED;

    // excellent bar
    if (connected) {
        if (rssi > -50) {
            color = ILI9341_GREEN;
        } else {
            color = ILI9341_LIGHTGREY;
        }
    }
    tft.fillRect(x + (2 * pixelSize), y, 5 * pixelSize, pixelSize, color);
    tft.fillRect(x + (1 * pixelSize), y + 1 * pixelSize, 1 * pixelSize, 1 * pixelSize, color);
    tft.fillRect(x + (7 * pixelSize), y + 1 * pixelSize, 1 * pixelSize, 1 * pixelSize, color);
    tft.fillRect(x + (0 * pixelSize), y + 2 * pixelSize, 1 * pixelSize, 1 * pixelSize, color);
    tft.fillRect(x + (8 * pixelSize), y + 2 * pixelSize, 1 * pixelSize, 1 * pixelSize, color);

    // fair bar
    if (connected) {
        if (rssi > -70) {
            color = ILI9341_GREEN;
        } else {
            color = ILI9341_LIGHTGREY;
        }
    }
    tft.fillRect(x + (3 * pixelSize), y + 2 * pixelSize, 3 * pixelSize, 1 * pixelSize, color);
    tft.fillRect(x + (2 * pixelSize), y + 3 * pixelSize, 1 * pixelSize, 1 * pixelSize, color);
    tft.fillRect(x + (6 * pixelSize), y + 3 * pixelSize, 1 * pixelSize, 1 * pixelSize, color);

    // weak bar
    if (connected) {
        if (rssi > -80) {
            color = ILI9341_GREEN;
        } else {
            color = ILI9341_LIGHTGREY;
        }
    }
    tft.fillRect(x + (4 * pixelSize), y + 4 * pixelSize, 1 * pixelSize, 1 * pixelSize, color);
    tft.fillRect(x + (3 * pixelSize), y + 5 * pixelSize, 3 * pixelSize, 1 * pixelSize, color);
    tft.fillRect(x + (4 * pixelSize), y + 6 * pixelSize, 1 * pixelSize, 1 * pixelSize, color);
}

void drawStartView() {
    tft.setCursor(35, 50);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(3);
    tft.println(F("DCC++ Throttle"));
    tft.setTextSize(2);
    tft.setCursor(136, 95);
    tft.println(VERSION);
}

void drawStartViewContinue() {
    tft.setTextSize(2);
    tft.setCursor(60, 130);
    tft.setTextColor(ILI9341_YELLOW);
    tft.println(F("Press any button"));
}

void drawMenuView() {
    drawMenuItems();
}

void drawTrainView() {
    // Draw Train Row
    drawTrainRow(currentTrain);

    // Draw footer
    drawFooterButtonsLabel((char *)"Next Trn", (char *)"Next Fnc", (char *)"Select");
}

int calculateXToCenter(char *text, byte fontWidth) {
    int widthOfText = strlen(text) * fontWidth;
    int x = SCREEN_WIDTH / 2 - widthOfText / 2;
    return x;
}

void drawTurnoutView() {
    char title[] = "Turnouts";
    int titleX = calculateXToCenter(title, FONT_SIZE_2_WIDTH);
    tft.setCursor(titleX, MAIN_VIEW_TOP_Y);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);
    tft.print(title);

    drawBlockView(TURNOUT_ROWS, TURNOUT_COLUMNS, &turnoutList, selectedTurnout, previousTurnout, true);
}

void drawAccessoriesView() {
    char title[] = "Accessories";
    int titleX = calculateXToCenter(title, FONT_SIZE_2_WIDTH);
    tft.setCursor(titleX, MAIN_VIEW_TOP_Y);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);
    tft.print(title);

    drawBlockView(ACCESSORY_ROWS, ACCESSORY_COLUMNS, &accessoryList, selectedAccessory, previousAccessory, true);
}

void drawSensorsView() {
    char title[] = "Sensors";
    int titleX = calculateXToCenter(title, FONT_SIZE_2_WIDTH);
    tft.setCursor(titleX, MAIN_VIEW_TOP_Y);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);
    tft.print(title);

    drawBlockView(SENSOR_ROWS, SENSOR_COLUMNS, &sensorList, selectedSensor, previousSensor, true);
}

void drawPreferencesView() {
    char title[] = "Preferences";
    int titleX = calculateXToCenter(title, FONT_SIZE_2_WIDTH);
    tft.setCursor(titleX, MAIN_VIEW_TOP_Y);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);
    tft.println(title);
}

template <typename T>
void drawBlockView(uint8_t numberOfRows, uint8_t numberOfColumns, LinkedList<T> *list, byte selectedIndex, byte lastIndex, bool drawAll) {
    byte count = 0;
    uint8_t leftAndRightEdgePadding = 20; // sum of left and right padding
    uint8_t displayBoxHeight = 162;       // available height for blocks
    uint8_t padding = 10;                 // horizontal and vertical padding between blocks
    int boxSpaceAvailableY = displayBoxHeight - ((numberOfRows - 1) * padding);
    uint8_t boxHeight = boxSpaceAvailableY / numberOfRows;
    int boxSpaceAvailableX = SCREEN_WIDTH - (((numberOfColumns - 1) * padding) + (leftAndRightEdgePadding));
    uint8_t boxWidth = boxSpaceAvailableX / numberOfColumns;

    int blockBackground = tft.color565(35, 35, 35);

    for (byte row = 0; row < numberOfRows; row++) {
        for (byte column = 0; column < numberOfColumns; column++) {
            if (count < list->size()) {
                // Only draw the block if drawing the whole view, or if this is the previous or current selection
                if (drawAll || count == selectedIndex || count == lastIndex) {
                    Component *thisComponent = list->get(count);
                    int thisBlockX = padding + (boxWidth * column) + (padding * column);
                    int thisBlockY = MAIN_VIEW_TOP_Y + FONT_SIZE_2_HEIGHT + padding + (row * boxHeight) + (row * padding);

                    // If this was the previously selected block, black out the highlight around the block
                    if (count == lastIndex) {
                        tft.drawRect(thisBlockX + 1, thisBlockY + 1, boxWidth - 2, boxHeight - 2, blockBackground);
                    }

                    // If this is the current selection, highlight the block
                    if (count == selectedIndex) {
                        tft.fillRect(thisBlockX, thisBlockY, boxWidth, boxHeight, blockBackground);
                        tft.drawRect(thisBlockX, thisBlockY, boxWidth, boxHeight, ILI9341_WHITE);
                        tft.drawRect(thisBlockX + 1, thisBlockY + 1, boxWidth - 2, boxHeight - 2, ILI9341_WHITE);
                    } else {
                        // Draw the normal box around this block
                        //tft.drawRect(thisBlockX, thisBlockY, boxWidth, boxHeight, ILI9341_WHITE);
                        tft.fillRect(thisBlockX, thisBlockY, boxWidth, boxHeight, blockBackground);
                    }

                    // TODO: fix
                    char *textUpperLeft = list->get(count)->getName();
                    tft.setTextColor(ILI9341_WHITE);
                    tft.setCursor((thisBlockX + ((boxWidth / 2) - ((strlen(textUpperLeft) * FONT_SIZE_2_WIDTH) / 2))), thisBlockY + 4);
                    tft.print(textUpperLeft);

                    byte innerPaddingX = boxWidth / 8;
                    byte innerPaddingY = boxHeight / 4;
                    if (thisComponent->isActive()) {
                        tft.fillRect(thisBlockX + innerPaddingX, thisBlockY + FONT_SIZE_2_HEIGHT + 6, innerPaddingX * 6, innerPaddingY * 2, ILI9341_DARKGREEN);
                        const char lOn[] = "On";
                        tft.setCursor((thisBlockX + ((boxWidth / 2) - ((strlen(lOn) * FONT_SIZE_2_WIDTH) / 2))), thisBlockY + FONT_SIZE_2_HEIGHT + 10);
                        tft.setTextColor(ILI9341_WHITE);
                        tft.print(lOn);
                    } else {
                        tft.fillRect(thisBlockX + innerPaddingX, thisBlockY + FONT_SIZE_2_HEIGHT + 6, innerPaddingX * 6, innerPaddingY * 2, ILI9341_DARKRED);
                        const char lOff[] = "Off";
                        tft.setTextColor(ILI9341_WHITE);
                        tft.setCursor((thisBlockX + ((boxWidth / 2) - ((strlen(lOff) * FONT_SIZE_2_WIDTH) / 2))), thisBlockY + FONT_SIZE_2_HEIGHT + 10);
                        tft.print(lOff);
                    }
                }
            }
            count++;
        }
    }
}

void drawFooterButtonsLabel(char *button1Label, char *button2Label, char *button3Label) {

    strcpy(footerButtonsLabel[FOOTER_BUTTON_LEFT], button1Label);
    strcpy(footerButtonsLabel[FOOTER_BUTTON_MIDDLE], button2Label);
    strcpy(footerButtonsLabel[FOOTER_BUTTON_RIGHT], button3Label);

    int footerYPosition = 219;
    int buttonWidth = 100;
    int buttonHeight = 21;
    int cornerRadius = 3;
    int widthOfLetter = 6;
    int paddingBetweenButtons = 10;

    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);

    if (strcmp(footerButtonsLabel[FOOTER_BUTTON_LEFT], "")) {
        tft.fillRoundRect(0, footerYPosition, buttonWidth, buttonHeight, cornerRadius, FOOTER_BUTTON_COLOR);
        int xPosition = buttonWidth / 2 - (strlen(footerButtonsLabel[FOOTER_BUTTON_LEFT]) * widthOfLetter);
        tft.setCursor(xPosition, footerYPosition + 3);
        tft.println(footerButtonsLabel[FOOTER_BUTTON_LEFT]);
    }

    if (strcmp(footerButtonsLabel[FOOTER_BUTTON_MIDDLE], "")) {
        tft.fillRoundRect(110, footerYPosition, buttonWidth, buttonHeight, cornerRadius, FOOTER_BUTTON_COLOR);
        int xPosition = buttonWidth / 2 - (strlen(footerButtonsLabel[FOOTER_BUTTON_MIDDLE]) * widthOfLetter);
        tft.setCursor(buttonWidth + paddingBetweenButtons + xPosition, footerYPosition + 3);
        tft.println(footerButtonsLabel[FOOTER_BUTTON_MIDDLE]);
    }

    if (strcmp(footerButtonsLabel[FOOTER_BUTTON_RIGHT], "")) {
        tft.fillRoundRect(220, footerYPosition, buttonWidth, buttonHeight, cornerRadius, FOOTER_BUTTON_COLOR);
        int xPosition = buttonWidth / 2 - (strlen(footerButtonsLabel[FOOTER_BUTTON_RIGHT]) * widthOfLetter);
        tft.setCursor((buttonWidth + paddingBetweenButtons) * 2 + xPosition, footerYPosition + 3);
        tft.println(footerButtonsLabel[FOOTER_BUTTON_RIGHT]);
    }
}

void footerButtonHighlighted(int button, bool isHighlighted) {
    char *buttonLabel = footerButtonsLabel[button];
    if (strcmp(buttonLabel, "")) {
        int footerYPosition = 219;
        int buttonWidth = 100;
        int buttonHeight = 21;
        int cornerRadius = 3;
        int widthOfLetter = 6;
        int paddingBetweenButtons = 10;

        int xPositionPadding = 0;
        if (button == FOOTER_BUTTON_MIDDLE) {
            xPositionPadding = buttonWidth + paddingBetweenButtons;
        } else if (button == FOOTER_BUTTON_RIGHT) {
            xPositionPadding = (buttonWidth + paddingBetweenButtons) * 2;
        }

        int backgroundColor = FOOTER_BUTTON_COLOR;
        if (isHighlighted) {
            backgroundColor = FOOTER_BUTTON_HIGHLIGHT_COLOR;
        }

        tft.fillRoundRect(xPositionPadding, footerYPosition, buttonWidth, buttonHeight, cornerRadius, backgroundColor);
        int xPosition = buttonWidth / 2 - (strlen(buttonLabel) * widthOfLetter);
        tft.setCursor(xPositionPadding + xPosition, footerYPosition + 3);
        tft.setTextColor(ILI9341_WHITE);
        tft.setTextSize(2);
        tft.println(buttonLabel);
        tft.setTextSize(1); // TODO: figure out how to reset text size
    }
}

void drawTrainRow(Train *train) {
    drawTrainNumber(train->address);
    drawTrainSpeed(train->getSpeed(), train->direction);
    drawTrainDirection(train->direction);
    clearFunctionRows();
    drawTrainFunctions(train);
}

void clearFunctionRows() {
    tft.fillRect(0, MAIN_VIEW_TOP_Y + 35, 320, 140, ILI9341_BLACK);
}

void drawTrainNumber(int trainNumber) {
    int trainNumberPositionTopLeftX = MAIN_VIEW_TOP_Y;

    //tft.fillRect(12, trainNumberPositionTopLeftX + 2, 64, 22, ILI9341_BLACK); // fill in the background of the box so we can overwrite it
    tft.drawRect(10, trainNumberPositionTopLeftX, 68, 26, ILI9341_WHITE);
    tft.drawRect(11, trainNumberPositionTopLeftX + 1, 66, 24, ILI9341_WHITE);

    tft.setCursor(16, trainNumberPositionTopLeftX + 6);
    tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
    tft.setTextSize(2);
    char trainNumberValue[6] = "";
    sprintf(trainNumberValue, "%05d", trainNumber);
    tft.println(trainNumberValue);
}

void drawTrainSpeed(int speed, byte direction) {
    tft.setCursor(85, MAIN_VIEW_TOP_Y + 6);
    tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
    tft.setTextSize(2);
    tft.print("Speed:");
    char output[4] = "";
    sprintf(output, "%03d", speed);

    tft.setCursor(85 + (6 * FONT_SIZE_2_WIDTH), MAIN_VIEW_TOP_Y + 6);
    tft.print(output);
    /*tft.fillRect(85 + (6 * FONT_SIZE_2_WIDTH), MAIN_VIEW_TOP_Y + 6, FONT_SIZE_2_WIDTH, FONT_SIZE_2_HEIGHT, ILI9341_BLACK);
    tft.setCursor(85 + (6 * FONT_SIZE_2_WIDTH), MAIN_VIEW_TOP_Y + 6);
    tft.print(output[0]);

    tft.fillRect(85 + (7 * FONT_SIZE_2_WIDTH), MAIN_VIEW_TOP_Y + 6, FONT_SIZE_2_WIDTH, FONT_SIZE_2_HEIGHT, ILI9341_BLACK);
    tft.setCursor(85 + (7 * FONT_SIZE_2_WIDTH), MAIN_VIEW_TOP_Y + 6);
    tft.print(output[1]);

    tft.fillRect(85 + (8 * FONT_SIZE_2_WIDTH), MAIN_VIEW_TOP_Y + 6, FONT_SIZE_2_WIDTH, FONT_SIZE_2_HEIGHT, ILI9341_BLACK);
    tft.setCursor(85 + (8 * FONT_SIZE_2_WIDTH), MAIN_VIEW_TOP_Y + 6);
    tft.print(output[2]);*/

    sprintf(previousSpeed, "%03d", speed);

    ////
    int xIncrement = 7;
    int speedGraphValue = speed / 21;

    tft.fillRect(80 + (10 * FONT_SIZE_2_WIDTH) + (xIncrement * 0), MAIN_VIEW_TOP_Y + 6 + 12, 5, 2, (speed > 0 ? ILI9341_GREEN : ILI9341_DARKGREY));
    tft.fillRect(80 + (10 * FONT_SIZE_2_WIDTH) + (xIncrement * 1), MAIN_VIEW_TOP_Y + 6 + 10, 5, 4, (speedGraphValue >= 1 ? ILI9341_GREEN : ILI9341_DARKGREY));
    tft.fillRect(80 + (10 * FONT_SIZE_2_WIDTH) + (xIncrement * 2), MAIN_VIEW_TOP_Y + 6 + 8, 5, 6, (speedGraphValue >= 2 ? ILI9341_GREEN : ILI9341_DARKGREY));
    tft.fillRect(80 + (10 * FONT_SIZE_2_WIDTH) + (xIncrement * 3), MAIN_VIEW_TOP_Y + 6 + 6, 5, 8, (speedGraphValue >= 3 ? ILI9341_GREEN : ILI9341_DARKGREY));
    tft.fillRect(80 + (10 * FONT_SIZE_2_WIDTH) + (xIncrement * 4), MAIN_VIEW_TOP_Y + 6 + 4, 5, 10, (speedGraphValue >= 4 ? ILI9341_GREEN : ILI9341_DARKGREY));
    tft.fillRect(80 + (10 * FONT_SIZE_2_WIDTH) + (xIncrement * 5), MAIN_VIEW_TOP_Y + 6 + 2, 5, 12, (speedGraphValue >= 5 ? ILI9341_GREEN : ILI9341_DARKGREY));
    tft.fillRect(80 + (10 * FONT_SIZE_2_WIDTH) + (xIncrement * 6), MAIN_VIEW_TOP_Y + 6 + 0, 5, 14, (speedGraphValue == 6 ? ILI9341_GREEN : ILI9341_DARKGREY));
}

void drawTrainDirection(byte direction) {
    //tft.fillRect(260, MAIN_VIEW_TOP_Y + 6, 36, 16, ILI9341_BLACK);
    tft.setCursor(260, MAIN_VIEW_TOP_Y + 6);
    tft.setTextSize(2);
    if (direction == TRAIN_FORWARD) {
        tft.setTextColor(ILI9341_DARKGREEN, ILI9341_BLACK);
        tft.print("FWD");
    } else {
        tft.setTextColor(ILI9341_DARKRED, ILI9341_BLACK);
        tft.print("REV");
    }
}

void drawTrainFunctions(Train *train) {
    // F0 Light
    // F1 Bell
    // F2 Horn
    byte topBuffer = 35;
    byte leftBuffer = 11;
    byte boxWidth = 34;
    byte boxHeight = 18;
    byte xBuffer = 10;
    byte yBuffer = 10;

    tft.setTextColor(ILI9341_WHITE);

    byte functionNumber = 0;

    // Set background color depending on the state
    int buttonColor = ILI9341_DARKRED;
    if (bitRead(train->functions, functionNumber)) {
        buttonColor = ILI9341_DARKGREEN;
    }
    tft.fillRect(leftBuffer, (MAIN_VIEW_TOP_Y + topBuffer), boxWidth, boxHeight, buttonColor);
    // Highlight if this is the selected function box
    if (train->activeFunction == 0) {
        tft.drawRect(leftBuffer, (MAIN_VIEW_TOP_Y + topBuffer), boxWidth, boxHeight, ILI9341_WHITE);
        tft.drawRect(leftBuffer + 1, (MAIN_VIEW_TOP_Y + topBuffer + 1), boxWidth - 2, boxHeight - 2, ILI9341_WHITE);
    }

    tft.setCursor(leftBuffer + 11, MAIN_VIEW_TOP_Y + topBuffer + 6);
    tft.setTextSize(1);
    tft.print("F");
    tft.print(functionNumber);
    functionNumber++;

    topBuffer = topBuffer + yBuffer;

    Serial.println(train->activeFunction);
    bool allFunctionsDrawn = false;
    for (byte row = 0; row < 4 && !allFunctionsDrawn; row++) {
        for (byte column = 0; column < 7 && !allFunctionsDrawn; column++) {
            if (functionNumber < train->numberOfFunctions) {
                // Set background color depending on the state
                int buttonColor = ILI9341_DARKRED;
                if (bitRead(train->functions, functionNumber)) {
                    buttonColor = ILI9341_DARKGREEN;
                }
                tft.fillRect((leftBuffer + ((boxWidth + xBuffer) * column)), (boxHeight + MAIN_VIEW_TOP_Y + topBuffer + (row * (boxHeight + yBuffer))), boxWidth, boxHeight, buttonColor);
                // Highlight if this is the selected function box
                if (train->activeFunction == functionNumber) {
                    tft.drawRect((leftBuffer + ((boxWidth + xBuffer) * column)), (boxHeight + MAIN_VIEW_TOP_Y + topBuffer + (row * (boxHeight + yBuffer))), boxWidth, boxHeight, ILI9341_WHITE);
                    tft.drawRect((leftBuffer + ((boxWidth + xBuffer) * column)) + 1, (boxHeight + MAIN_VIEW_TOP_Y + topBuffer + (row * (boxHeight + yBuffer))) + 1, boxWidth - 2, boxHeight - 2, ILI9341_WHITE);
                }

                byte stringXBuffer = 11;
                if (functionNumber > 9) {
                    stringXBuffer = 9;
                }
                tft.setCursor((leftBuffer + ((boxWidth + xBuffer) * column)) + stringXBuffer, boxHeight + (MAIN_VIEW_TOP_Y + topBuffer + (row * (boxHeight + yBuffer))) + 6);
                tft.print("F");
                tft.print(functionNumber);
                functionNumber++;
            } else {
                allFunctionsDrawn = true;
            }
        }
    }
    previouslySelectedFunctionNumber = train->activeFunction;
}

void updateTrainFunctions(Train *train) {

    byte topBuffer = 35;
    byte leftBuffer = 11;
    byte boxWidth = 34;
    byte boxHeight = 18;
    byte xBuffer = 10;
    byte yBuffer = 10;

    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);

    byte functionNumber = 0;

    if (previouslySelectedFunctionNumber == 0) {
        int buttonColor = ILI9341_DARKRED;
        if (bitRead(train->functions, functionNumber)) {
            buttonColor = ILI9341_DARKGREEN;
        }
        tft.fillRect(leftBuffer, (MAIN_VIEW_TOP_Y + topBuffer), boxWidth, boxHeight, buttonColor);
        tft.setCursor(leftBuffer + 11, MAIN_VIEW_TOP_Y + topBuffer + 6);
        tft.print("F");
        tft.print(functionNumber);
    }

    if (train->activeFunction == functionNumber) {
        tft.drawRect(leftBuffer, (MAIN_VIEW_TOP_Y + topBuffer), boxWidth, boxHeight, ILI9341_WHITE);
        tft.drawRect(leftBuffer + 1, (MAIN_VIEW_TOP_Y + topBuffer + 1), boxWidth - 2, boxHeight - 2, ILI9341_WHITE);
    }

    functionNumber++;
    topBuffer = topBuffer + yBuffer;

    Serial.println(train->activeFunction);
    bool allFunctionsDrawn = false;

    for (byte row = 0; row < 4 && !allFunctionsDrawn; row++) {
        for (byte column = 0; column < 7 && !allFunctionsDrawn; column++) {
            if (functionNumber < train->numberOfFunctions) {
                if (previouslySelectedFunctionNumber == functionNumber) {
                    int buttonColor = ILI9341_DARKRED;
                    if (bitRead(train->functions, functionNumber)) {
                        buttonColor = ILI9341_DARKGREEN;
                    }
                    tft.fillRect((leftBuffer + ((boxWidth + xBuffer) * column)), (boxHeight + MAIN_VIEW_TOP_Y + topBuffer + (row * (boxHeight + yBuffer))), boxWidth, boxHeight, buttonColor);

                    byte stringXBuffer = 11;
                    if (functionNumber > 9) {
                        stringXBuffer = 9;
                    }
                    tft.setCursor((leftBuffer + ((boxWidth + xBuffer) * column)) + stringXBuffer, boxHeight + (MAIN_VIEW_TOP_Y + topBuffer + (row * (boxHeight + yBuffer))) + 6);
                    tft.print("F");
                    tft.print(functionNumber);
                }
                if (train->activeFunction == functionNumber) {
                    tft.drawRect((leftBuffer + ((boxWidth + xBuffer) * column)), (boxHeight + MAIN_VIEW_TOP_Y + topBuffer + (row * (boxHeight + yBuffer))), boxWidth, boxHeight, ILI9341_WHITE);
                    tft.drawRect((leftBuffer + ((boxWidth + xBuffer) * column)) + 1, (boxHeight + MAIN_VIEW_TOP_Y + topBuffer + (row * (boxHeight + yBuffer))) + 1, boxWidth - 2, boxHeight - 2, ILI9341_WHITE);
                }
                functionNumber++;
            } else {
                allFunctionsDrawn = true;
            }
        }
    }

    previouslySelectedFunctionNumber = train->activeFunction;
}

void clearView() {
    tft.fillRect(0, MAIN_VIEW_TOP_Y, 320, 220, ILI9341_BLACK);
}

void drawMenuItems() {
    // clear menu
    tft.fillRect(0, 30, 320, 150, ILI9341_BLACK);

    tft.setCursor(0, MAIN_VIEW_TOP_Y);
    tft.setTextSize(2);
    for (byte i = 0; i < NUMBER_OF_MENU_ITEMS; i++) {
        if (i == menuViewSelection) {
            tft.setTextColor(ILI9341_BLACK);
            tft.fillRect(0, MAIN_VIEW_TOP_Y + i * FONT_SIZE_2_HEIGHT, 320, FONT_SIZE_2_HEIGHT, ILI9341_WHITE);
        } else {
            tft.setTextColor(ILI9341_WHITE);
            tft.fillRect(0, MAIN_VIEW_TOP_Y + i * FONT_SIZE_2_HEIGHT, 320, FONT_SIZE_2_HEIGHT, ILI9341_BLACK);
        }
        tft.println(menuText[i]);
    }
    previousMenuViewSelection = menuViewSelection;
}

void drawMenuItem() {
    clearPreviousMenuSelection();
    drawCurrentMenuSelection();
    previousMenuViewSelection = menuViewSelection;
}

void drawCurrentMenuSelection() {
    for (byte i = 0; i < NUMBER_OF_MENU_ITEMS; i++) {
        if (i == menuViewSelection) {
            tft.fillRect(0, MAIN_VIEW_TOP_Y + i * FONT_SIZE_2_HEIGHT, 320, FONT_SIZE_2_HEIGHT, ILI9341_WHITE);
            tft.setCursor(0, MAIN_VIEW_TOP_Y + i * FONT_SIZE_2_HEIGHT);
            tft.setTextColor(ILI9341_BLACK);
            tft.setTextSize(2);
            tft.println(menuText[i]);
        }
    }
}

void clearPreviousMenuSelection() {
    for (byte i = 0; i < NUMBER_OF_MENU_ITEMS; i++) {
        if (i == previousMenuViewSelection) {
            tft.fillRect(0, MAIN_VIEW_TOP_Y + i * FONT_SIZE_2_HEIGHT, 320, FONT_SIZE_2_HEIGHT, ILI9341_BLACK);
            tft.setCursor(0, MAIN_VIEW_TOP_Y + i * FONT_SIZE_2_HEIGHT);
            tft.setTextColor(ILI9341_WHITE);
            tft.setTextSize(2);
            tft.println(menuText[i]);
        }
    }
}

void drawTrackPower() {
    tft.setCursor(FONT_SIZE_2_WIDTH * 8 + 5, 5);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);

    tft.fillRect(FONT_SIZE_2_WIDTH * 8 + 3, 3, FONT_SIZE_2_WIDTH * 3 + 2, FONT_SIZE_2_HEIGHT + 2, ILI9341_BLACK);
    if (trackPower == TRACK_POWER_ON) {
        tft.fillRect(FONT_SIZE_2_WIDTH * 8 + 3, 3, FONT_SIZE_2_WIDTH * 2 + 2, FONT_SIZE_2_HEIGHT + 2, ILI9341_GREEN);
        tft.setTextColor(ILI9341_BLACK);
        tft.println("ON");
    } else {
        tft.fillRect(FONT_SIZE_2_WIDTH * 8 + 3, 3, FONT_SIZE_2_WIDTH * 3 + 2, FONT_SIZE_2_HEIGHT + 2, ILI9341_RED);
        tft.setTextColor(ILI9341_BLACK);
        tft.println("OFF");
    }
}

void drawTrackCurrent() {
    if (displayState != START) {
        tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
        tft.setCursor(FONT_SIZE_2_WIDTH * 13, 5);
        tft.setTextSize(2);
        char amps[5] = "";
        sprintf(amps, "%d.%02d", (int)trackCurrent, (int)(trackCurrent * 100) % 100);
        tft.println(amps);
    }
}

void buttonScan() {

    static byte previousDisplayState[NUMBER_OF_BUTTONS];
    static byte displayState[NUMBER_OF_BUTTONS];
    static unsigned long lastTime;

    if ((lastTime + BUTTON_DEBOUNCE_MS) > millis()) {
        // not enough time has passed to debounce
        return;
    }

    // ok we have waited BUTTON_DEBOUNCE_MS milliseconds, lets reset the timer
    lastTime = millis();

    for (byte i = 0; i < sizeof(buttons); i++) {
        displayState[i] = digitalRead(buttons[i]); // read the button

        if (displayState[i] == previousDisplayState[i]) {
            if ((pressedButton[i] == LOW) && (displayState[i] == LOW)) {
                // just pressed
                justPressedButton[i] = 1;
                if (i == 1 || i == 2 || i == 3) {
                    footerButtonHighlighted(i - 1, true);
                }
            } else if ((pressedButton[i] == HIGH) && (displayState[i] == HIGH)) {
                // just released
                justReleasedButton[i] = 1;
                if (i == 1 || i == 2 || i == 3) {
                    footerButtonHighlighted(i - 1, false);
                }
            }
            pressedButton[i] = !displayState[i]; // remember, digital HIGH means NOT pressed
        }
        previousDisplayState[i] = displayState[i]; // keep a running tally of the buttons
    }
}

void toggleTrackPower() {
    trackPower = !trackPower;
}

void sendTrackPowerCommand() {
    if (trackPower == TRACK_POWER_ON) {
        send((char *)"<1>");
    } else {
        rotaryEncoder.setEncoderValue(0);
        for (byte i = 0; i < trainList.size(); i++) {
            trainList.get(i)->setSpeed(0);
            char stopCommand[19];
            trainList.get(i)->getEmergencyStopCommand(stopCommand);
            send(stopCommand);
        }
        send((char *)"<0>");
        speedIndicatorValue = 0;
        xTaskCreate(moveGauge, "MoveGauge", 1024, (void *)&speedIndicatorValue, 1, NULL);
    }
}

void resetGauge(void *parameter) {
    xSemaphoreTake(speedIndicatorSemaphore, portMAX_DELAY);
    speedIndicator->reset();
    xSemaphoreGive(speedIndicatorSemaphore);
    vTaskDelete(NULL);
}

void moveGauge(void *parameter) {
    int8_t value = *((int8_t *)parameter);
    xSemaphoreTake(speedIndicatorSemaphore, portMAX_DELAY);
    speedIndicator->move(value);
    xSemaphoreGive(speedIndicatorSemaphore);
    vTaskDelete(NULL);
}

void sendTrainCommand(Train *train) {
    static char speedCommand[19];
    train->getSpeedCommand(speedCommand);
    send(speedCommand);
    speedIndicatorValue = currentTrain->getSpeed();
    if (currentTrain->getDirection() != 1) {
        speedIndicatorValue = -speedIndicatorValue;
    }
    xTaskCreate(moveGauge, "MoveGauge", 1024, (void *)&speedIndicatorValue, 1, NULL);
}

void sendCurrentRequestCommand() {
    send((char *)"<c>");
}

void parseCommand(char *command) {

    switch (command[0]) {

        case 'T':
            parseTCommand(command);
            break;

        case 'H':
            parseHCommand(command);
            break;

        case 'p':
            if (command[1] == '0') {
                trackPower = TRACK_POWER_OFF;
            } else if (command[1] == '1') {
                trackPower = TRACK_POWER_ON;
            } else if (command[1] == '2' || command[1] == '3') {
                trackPower = TRACK_POWER_OFF;
            }
            if (displayState != START) {
                drawTrackPower();
            }
            break;

        case 'a':
            // remove the 'a' from the start of the value
            //<a val>
            //<c MeterName value C/V unit min max res warn>
            char *commandTrimmed = &command[1];
            float cur = atof(commandTrimmed) / 100;
            trackCurrentSamples.add(cur);
            trackCurrent = trackCurrentSamples.getAverage();
            break;
    }
}

/*void parseX(char *command) {

    int trainRegister = 0;
    int speed = 0;
    byte direction = 0;

    Serial.print("command: ");
    Serial.println(&command[1]);
    //sscanf(&command[1], "%d %d %d", &trainRegister, &speed, &direction);
}*/

void parseTCommand(char *command) {
    //Serial.print(F("Ignoring T command: "));
    //Serial.println(command);
}

void parseHCommand(char *command) {
    //byte hMessageType = 0;
    int id = 0;
    int address = 0;
    int subAddress = 0;
    int thrown = -1;

    sscanf(&command[1], "%d %d %d %d", &id, &address, &subAddress, &thrown);

    Serial.print("id: ");
    Serial.println(id);
    Serial.print("address: ");
    Serial.println(address);
    Serial.print("subAddress: ");
    Serial.println(subAddress);
    Serial.print("thrown: ");
    Serial.println(thrown);

    if (thrown == -1) {
        Serial.println("shortened version");
    } else {
        Serial.println("long version");
    }
}
