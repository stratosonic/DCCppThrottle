#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include "SPI.h"
#include <Arduino.h>
#include <LinkedList.h>

#if defined ESP32
#include <WiFi.h>
#include <HTTPClient.h>
#endif

#include "Accessory/Accessory.h"
#include "Comm/Comm.h"
#include "Config.h"
#include "Sensor/Sensor.h"
#include "Train/Train.h"
#include "Turnout/Turnout.h"

#include "Graphics.c"

// clang-format off

// Set to 1 for extra debug messages to serial out
#define DEBUG 0

//#define VERSION "V0.1"

#if defined ESP32
hw_timer_t * timer = NULL;

//const char* ssid = SSID_NAME;
//const char* password = SSID_PASSWORD;

#endif

// Use hardware SPI
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS_PIN, TFT_DC_PIN, TFT_RST_PIN);

//tft.color565(0,0,0);
#define ILI9341_DARKRED 0xB000 /* 22, 0, 0 */
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

String footerButtonsLabel[] = {"", "", ""};

#define BUTTON_DEBOUNCE_MS       10 // how many ms to debounce

#define ENCODER_BUTTON_PIN       ENCODER_PIN
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

#if defined ESP32
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE mutex = portMUX_INITIALIZER_UNLOCKED;
#endif

volatile boolean encoderHalfLeft = false; // Used in both interrupt routines
volatile boolean encoderHalfRight = false;
volatile int encoderLastChange = 0;
volatile boolean encoderAccelerated = false;

volatile byte encoderCount = 0;
volatile byte lastEncoded = 0;

Comm *comm = new Comm();
//Comm::mutex = portMUX_INITIALIZER_UNLOCKED;
//portMUX_TYPE Comm::sendMutex = portMUX_INITIALIZER_UNLOCKED;

#define MAX_COMMAND_LENGTH 255
char commandString[MAX_COMMAND_LENGTH];

#define TRACK_POWER_ON    1
#define TRACK_POWER_OFF   0
bool trackPower = TRACK_POWER_OFF;

// counter for how ofter to check track current
#define TRACK_CURRENT_REQUEST_FREQUENCY 100  // 100 = once per second
int trackCurrentCounter = 0;
float trackCurrent = 0.0;

// Previous speed value for display checking to avoid flicker
char previousSpeed[3];

byte menuViewSelection = 0;
byte previousMenuViewSelection = 0;
String menuText[] = {"Control Trains", "Turnouts", "Accessories", "Sensors", "Preferences"};
#define NUMBER_OF_MENU_ITEMS 5

// Maintain the order of the menu to match the menuText array
enum DisplayState { TRAIN_CONTROL, TURNOUT, ACCESSORIES, SENSORS, PREFERENCES, START, MENU };
DisplayState displayState = START;
DisplayState previousDisplayState = displayState;

enum EncoderState { NIL, NEXT, PREVIOUS };
EncoderState encoderState = NIL;

byte previouslySelectedFunctionNumber = 28;

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

char spinnerArray[] = {'-', '|', '/', '-', '\\', '|', '/'};
byte spinnerArrayPointer = 0;

// clang-format on

//////
#if defined ESP32
void onTimer1();
#endif

void drawSpinner();
void drawStartView();
void drawStartViewContinue();
void parseCommand(char *);
void updateEncoder();
void encoderPin1Interrupt();
void encoderPin2Interrupt();
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
void drawFooterButtonsLabel(String, String, String);
template <typename T>
void drawBlockView(uint8_t numberOfRows, uint8_t numberOfColumns, LinkedList<T> *list, byte selectedIndex, byte lastIndex, bool drawAll);
void drawTrainRow(Train *);
void drawTrainNumber(int);
void drawTrainDirection(byte);
void drawTrainFunctions(Train *);
void drawTrainSpeed(int, byte);
void updateTrainFunctions(Train *);
void drawTurnout(int, int, int, int, bool, String, bool);
void drawTurnout();
void drawTurnoutView();
void toggleTrackPower();
void sendTrackPowerCommand();
void sendCurrentRequestCommand();
void sendTrainCommand(Train *);
void drawTrackPower();
void drawTrackCurrent();
void drawWifiSybol(int, int, bool);
void drawBattery(int, int, byte);
void parseTCommand(char *command);
void parseHCommand(char *command);
Train *getTrainByRegister(byte trainRegister);
void enableTimerInterrupt();
void disableTimerInterrupt();
void readResponse(String response);
/////

HTTPClient *http;

void setup() {
    Serial.begin(115200);
    if (DEBUG == 1) {
        Serial.println(F("DCC++ Throttle"));
    }

    tft.begin();
    tft.setRotation(3);
    tft.fillScreen(ILI9341_BLACK);

    drawStartView();

    #if defined ESP32

    // First disconnect so reconnection will work
    WiFi.disconnect(false);

    //Delay approx 4 seconds before calling the WiFi.begin
    for(byte i = 0; i < 53; i++) {
        drawSpinner();
        delay(75);
    }

    WiFi.begin(SSID_NAME, SSID_PASSWORD);

    while (WiFi.status() != WL_CONNECTED) {   //Check for the connection
        drawSpinner();
        delay(75);
        Serial.println("Connecting to WiFi..");
    }

    Serial.println("Connected to the WiFi network");

    //WiFi.RSSI();

    #endif

    pinMode(ENCODER_PIN_1, INPUT_PULLUP);
    pinMode(ENCODER_PIN_2, INPUT_PULLUP);

#if defined ARDUINO_AVR_MEGA2560
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_1), encoderPin1Interrupt, FALLING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_2), encoderPin2Interrupt, FALLING);
#elif defined ESP32
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_1), encoderPin2Interrupt, FALLING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_2), encoderPin1Interrupt, FALLING);
#endif

    // Set ports for buttons to inputs with internal pullup
    for (byte i = 0; i < NUMBER_OF_BUTTONS; i++) {
        pinMode(buttons[i], INPUT_PULLUP);
    }

    // ----------  Add Trains  ---------- //
    Train *train1 = new Train(1, 8671, MAX_NUMBER_OF_TRAIN_FUNCTIONS);
    trainList.add(train1);
    Train *train2 = new Train(2, 1234, MAX_NUMBER_OF_TRAIN_FUNCTIONS);
    trainList.add(train2);
    Train *train3 = new Train(3, 10035, 8);
    trainList.add(train3);
    currentTrain = trainList.get(currentTrainIndex);

    // ----------  Add Turnouts  ---------- //
    Turnout *turnout1 = new Turnout(1, "Turnout 1");
    turnoutList.add(turnout1);
    Turnout *turnout2 = new Turnout(2, "Yard 1");
    turnoutList.add(turnout2);
    Turnout *turnout3 = new Turnout(3, "Yard 2");
    turnoutList.add(turnout3);
    Turnout *turnout4 = new Turnout(4, "Warehouse");
    turnoutList.add(turnout4);
    Turnout *turnout5 = new Turnout(5, "Extra A");
    turnoutList.add(turnout5);
    Turnout *turnout6 = new Turnout(6, "Extra B");
    turnoutList.add(turnout6);

    // ----------  Add Accessories  ---------- //
    accessoryList.add(new Accessory("A1", 1, 1));
    accessoryList.add(new Accessory("Light", 1, 2));
    accessoryList.add(new Accessory("Swing", 1, 3));
    accessoryList.add(new Accessory("Blah", 1, 4));

    // ----------  Add Sensors  ---------- //
    sensorList.add(new Sensor("Snsr 1"));
    sensorList.add(new Sensor("Snsr 2"));
    sensorList.add(new Sensor("S 3"));
    sensorList.add(new Sensor("S 4"));
    sensorList.add(new Sensor("S 5"));
    sensorList.add(new Sensor("S 6"));

    if (DEBUG == 1) {
        Serial.println(F("Done with setup"));
    }

    // Send status request
    readResponse(comm->send("<s>"));
    // Request all defined turnouts
    readResponse(comm->send("<T>"));
    // Request all defined Sensors
    readResponse(comm->send("<S>"));
    // Request all defined output pins
    readResponse(comm->send("<Z>"));
    // Status of all sensors ???
    readResponse(comm->send("<Q>"));

    #if defined ARDUINO_AVR_MEGA2560
    // http://www.8bit-era.cz/arduino-timer-interrupts-calculator.html
    // TIMER 1 for interrupt frequency 100 Hz:
    cli();     // stop interrupts
    TCCR1A = 0;     // set entire TCCR1A register to 0
    TCCR1B = 0;     // same for TCCR1B
    TCNT1 = 0;     // initialize counter value to 0
    // set compare match register for 100 Hz increments
    OCR1A = 19999;     // = 16000000 / (8 * 100) - 1 (must be <65536)
    // turn on CTC mode
    TCCR1B |= (1 << WGM12);
    // Set CS12, CS11 and CS10 bits for 8 prescaler
    TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);
    sei();     // allow interrupts

    #elif defined ESP32
    // https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/
    // TIMER 1 for interrupt frequency of 100 Hz:
    // Clock frequency: 80Mhz
    // 80,000,000 / 8000 / 100 = 100 Hz

    timer = timerBegin(0, 8000, true);     // 80000000 / 8000 (must be <65536) = 10000 Hz
    enableTimerInterrupt();

    #endif

    // hide spinner
    tft.fillRect(152, 130, FONT_SIZE_2_WIDTH, FONT_SIZE_2_HEIGHT, ILI9341_BLACK);

    drawStartViewContinue();
}

void enableTimerInterrupt() {
    timerAttachInterrupt(timer, &onTimer1, true);
    timerAlarmWrite(timer, 100, true);     // 10000 / 100 = 100Hz
    timerAlarmEnable(timer);
}

void disableTimerInterrupt() {
    timerDetachInterrupt(timer);
}

void drawSpinner() {
    tft.fillRect(152, 130, FONT_SIZE_2_WIDTH, FONT_SIZE_2_HEIGHT, ILI9341_BLACK);
    tft.setCursor(152, 130);
    tft.print(spinnerArray[spinnerArrayPointer]);
    spinnerArrayPointer++;
    if(spinnerArrayPointer >= 7) {
        spinnerArrayPointer = 0;
    }
}

void readResponse(String response) {
    Serial.print("Response: ");
    Serial.println(response);

    int bufferLength = response.length()+1;
    char buffer[bufferLength] = "";
    response.toCharArray(buffer, bufferLength);

    for(int i = 0; i < bufferLength; i++) {
        int c = buffer[i]; //Serial.read();

        if (c == '<') { // start of new command
            sprintf(commandString, "");
        } else if (c == '>') { // end of new command
            Serial.print("Parsing: ");
            Serial.println(commandString);
            parseCommand(commandString);
        } else if (strlen(commandString) < MAX_COMMAND_LENGTH) { // if comandString still has space, append character just read from serial line
            sprintf(commandString, "%s%c", commandString, c); // otherwise, character is ignored (but continue to look for '<' or '>')
        }
    }
}
void loop(void) {

    if (previousDisplayState != displayState) {
        if (DEBUG == 1) {
            Serial.print(F("displayState "));
            Serial.println(displayState);
        }
        previousDisplayState = displayState;
    }

    if (trackCurrentCounter >= 500) {
        disableTimerInterrupt();
        sendCurrentRequestCommand();
        trackCurrentCounter = 0;
        enableTimerInterrupt();
    }

    // Check track power button every loop
    if (displayState != START) {
        if (TRACK_POWER_BUTTON_JUST_PRESSED) {
            TRACK_POWER_BUTTON_JUST_PRESSED = 0;
            toggleTrackPower();
            sendTrackPowerCommand();
            drawTrackPower();
            if(displayState == TRAIN_CONTROL) {
                drawTrainSpeed(currentTrain->getSpeed(), currentTrain->direction);
            }
        }
    }

    byte currentTrainDirection = -1;

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
            comm->send(currentTrain->getFunctionCommand());
            updateTrainFunctions(currentTrain);
        } else if (MENU_BUTTON_JUST_PRESSED) {
            clearButtonStates();
            displayState = MENU;
            clearView();
            drawMenuView();
        }

        if (ENCODER_BUTTON_JUST_PRESSED) {
            clearButtonStates();

            if (currentTrain->getSpeed() > 0 || currentTrain->getSpeed() < 0) {
                currentTrain->setSpeed(0);
                drawTrainSpeed(currentTrain->getSpeed(), currentTrain->direction);
            } else if (currentTrain->getSpeed() == 0) {
                currentTrain->direction = !currentTrain->direction;
            }
            sendTrainCommand(currentTrain);
            drawTrainDirection(currentTrain->direction);
        }

        switch (encoderState) {
        case NEXT:
            clearEncoderState();
            currentTrainDirection = currentTrain->direction;
            if (encoderAccelerated) {
                currentTrain->increaseSpeed(10);
            } else {
                currentTrain->increaseSpeed(1);
            }
            drawTrainSpeed(currentTrain->getSpeed(), currentTrain->direction);
            if (currentTrainDirection != currentTrain->direction) {
                drawTrainDirection(currentTrain->direction);
            }
            sendTrainCommand(currentTrain);
            break;

        case PREVIOUS:
            clearEncoderState();
            currentTrainDirection = currentTrain->direction;
            if (encoderAccelerated) {
                currentTrain->decreaseSpeed(10);
            } else {
                currentTrain->decreaseSpeed(1);
            }
            drawTrainSpeed(currentTrain->getSpeed(), currentTrain->direction);
            if (currentTrainDirection != currentTrain->direction) {
                drawTrainDirection(currentTrain->direction);
            }
            sendTrainCommand(currentTrain);
            break;

        default:
            break;
        }
        break;

    // ---------------------- MENU ------------------------ //
    case MENU:

        switch (encoderState) {
        case PREVIOUS:
            clearEncoderState();
            if (menuViewSelection == 0) {
                menuViewSelection = NUMBER_OF_MENU_ITEMS - 1;
            } else {
                menuViewSelection--;
            }
            drawMenuItem();
            break;
        case NEXT:
            clearEncoderState();
            menuViewSelection++;
            if (menuViewSelection > NUMBER_OF_MENU_ITEMS - 1) {
                menuViewSelection = 0;
            }
            drawMenuItem();
            break;
        default:
            break;
        }

        if (ENCODER_BUTTON_JUST_PRESSED) {
            clearButtonStates();
            clearEncoderState();
            clearView();
            if (menuViewSelection == TRAIN_CONTROL) {
                displayState = TRAIN_CONTROL;
                drawTrainView();
            } else if (menuViewSelection == TURNOUT) {
                displayState = TURNOUT;
                drawTurnoutView();
            } else if (menuViewSelection == ACCESSORIES) {
                displayState = ACCESSORIES;
                drawAccessoriesView();
            } else if (menuViewSelection == SENSORS) {
                displayState = SENSORS;
                drawSensorsView();
            } else if (menuViewSelection == PREFERENCES) {
                displayState = PREFERENCES;
                drawPreferencesView();
            }
        }

        break;

    // ---------------------- TURNOUT ------------------------ //
    case TURNOUT:
        if (MENU_BUTTON_JUST_PRESSED) {
            clearButtonStates();
            displayState = MENU;
            clearView();
            drawMenuView();
        }

        switch (encoderState) {

        case PREVIOUS:
            clearEncoderState();
            selectedTurnout--;
            if (selectedTurnout < 0) {
                selectedTurnout = turnoutList.size() - 1;
            }
            drawBlockView(TURNOUT_ROWS, TURNOUT_COLUMNS, &turnoutList, selectedTurnout, previousTurnout, false);
            previousTurnout = selectedTurnout;
            break;
        case NEXT:
            clearEncoderState();
            selectedTurnout++;
            if (selectedTurnout > turnoutList.size() - 1) {
                selectedTurnout = 0;
            }

            drawBlockView(TURNOUT_ROWS, TURNOUT_COLUMNS, &turnoutList, selectedTurnout, previousTurnout, false);
            previousTurnout = selectedTurnout;
            break;
        default:
            break;
        }

        if (ENCODER_BUTTON_JUST_PRESSED) {
            clearButtonStates();
            Turnout *turnout = turnoutList.get(selectedTurnout);
            turnout->toggleActive();
            drawBlockView(TURNOUT_ROWS, TURNOUT_COLUMNS, &turnoutList, selectedTurnout, previousTurnout, false);
        }
        break;

    // ---------------------- ACCESSORIES ------------------------ //
    case ACCESSORIES:
        if (MENU_BUTTON_JUST_PRESSED) {
            clearButtonStates();
            displayState = MENU;
            clearView();
            drawMenuView();
        }

        switch (encoderState) {
        case PREVIOUS:
            clearEncoderState();
            if (selectedAccessory == 0) {
                selectedAccessory = accessoryList.size() - 1;
            } else {
                selectedAccessory--;
            }
            drawBlockView(ACCESSORY_ROWS, ACCESSORY_COLUMNS, &accessoryList, selectedAccessory, previousAccessory, false);
            previousAccessory = selectedAccessory;
            break;
        case NEXT:
            clearEncoderState();
            selectedAccessory++;
            if (selectedAccessory > accessoryList.size() - 1) {
                selectedAccessory = 0;
            }
            drawBlockView(ACCESSORY_ROWS, ACCESSORY_COLUMNS, &accessoryList, selectedAccessory, previousAccessory, false);
            previousAccessory = selectedAccessory;
            break;
        default:
            break;
        }

        if (ENCODER_BUTTON_JUST_PRESSED) {
            clearButtonStates();
            Accessory *accessory = accessoryList.get(selectedAccessory);
            accessory->toggleActive();
            drawBlockView(ACCESSORY_ROWS, ACCESSORY_COLUMNS, &accessoryList, selectedAccessory, previousAccessory, false);
        }
        break;

    // ----------------------- SENSORS ----------------------- //
    case SENSORS:
        if (MENU_BUTTON_JUST_PRESSED) {
            clearButtonStates();
            displayState = MENU;
            clearView();
            drawMenuView();
        }

        switch (encoderState) {
        case PREVIOUS:
            clearEncoderState();
            if (selectedSensor == 0) {
                selectedSensor = sensorList.size() - 1;
            } else {
                selectedSensor--;
            }
            drawBlockView(SENSOR_ROWS, SENSOR_COLUMNS, &sensorList, selectedSensor, previousSensor, false);
            previousSensor = selectedSensor;
            break;
        case NEXT:
            clearEncoderState();
            selectedSensor++;
            if (selectedSensor > sensorList.size() - 1) {
                selectedSensor = 0;
            }
            drawBlockView(SENSOR_ROWS, SENSOR_COLUMNS, &sensorList, selectedSensor, previousSensor, false);
            previousSensor = selectedSensor;
            break;
        default:
            break;
        }

        if (ENCODER_BUTTON_JUST_PRESSED) {
            clearButtonStates();
            Sensor *sensor = sensorList.get(selectedSensor);
            sensor->toggleActive();
            drawBlockView(SENSOR_ROWS, SENSOR_COLUMNS, &sensorList, selectedSensor, previousSensor, false);
        }
        break;

    // ----------------------- PREFERENCES ----------------------- //
    case PREFERENCES:
        if (MENU_BUTTON_JUST_PRESSED) {
            clearButtonStates();
            displayState = MENU;
            clearView();
            drawMenuView();
        }
        break;

    default:
        break;
    }
}

// Timer 1 interrupt service routine
#if defined ARDUINO_AVR_MEGA2560

ISR(TIMER1_COMPA_vect) {
    buttonScan();
    trackCurrentCounter++;
}

#elif defined ESP32

void onTimer1() {
    buttonScan();
    trackCurrentCounter++;
}
#endif

// http://bildr.org/2012/08/rotary-encoder-arduino/
/*void updateEncoder(){
    portENTER_CRITICAL(&mux);
    byte tempEncoderstate = ENCODER_STATE_NULL;

    byte MSB = digitalRead(ENCODER_PIN_1); //MSB = most significant bit
    byte LSB = digitalRead(ENCODER_PIN_2); //LSB = least significant bit

    byte encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
    byte sum = (lastEncoded << 2) | encoded; //adding it to the previous encoded value
    if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
        tempEncoderstate = PREVIOUS;
        encoderCount++;
    }
    if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
        tempEncoderstate = NEXT;
        encoderCount++;
    }
    lastEncoded = encoded; //store this value for next time
    if(encoderCount > 3) {
        encoderCount = 0;
        encoderState = tempEncoderstate;

        int millisSinceLastChange = millis() - encoderLastChange;
        if (millisSinceLastChange < 50) {
            encoderAccelerated = true;
        } else {
            encoderAccelerated = false;
        }
        encoderLastChange = millis();
    }
    portEXIT_CRITICAL(&mux);
   }*/

void encoderPin1Interrupt() { // Pin went LOW
    #if defined ARDUINO_AVR_MEGA2560
    delay(1);     // Debounce time
    #elif defined ESP32
    portENTER_CRITICAL(&mux);
    #endif

    if (digitalRead(ENCODER_PIN_1) == LOW) { // Pin still LOW ?
        if (digitalRead(ENCODER_PIN_2) == HIGH && encoderHalfRight == false) { // -->
            encoderHalfRight = true; // One half click clockwise
        }
        if (digitalRead(ENCODER_PIN_2) == LOW && encoderHalfLeft == true) { // <--

            int millisSinceLastChange = millis() - encoderLastChange;
            if (millisSinceLastChange < 50) {
                encoderAccelerated = true;
            } else {
                encoderAccelerated = false;
            }
            encoderLastChange = millis();
            encoderHalfLeft = false; // One whole click counter-
            encoderState = PREVIOUS;
        }
    }

    #if defined ESP32
    portEXIT_CRITICAL(&mux);
    #endif
}

void encoderPin2Interrupt() { // Pin went LOW
    #if defined ARDUINO_AVR_MEGA2560
    delay(1);     // Debounce time
    #elif defined ESP32
    portENTER_CRITICAL(&mux);
    #endif

    if (digitalRead(ENCODER_PIN_2) == LOW) { // Pin still LOW ?
        if (digitalRead(ENCODER_PIN_1) == HIGH && encoderHalfLeft == false) { // <--
            encoderHalfLeft = true; // One half  click counter-
        } // clockwise
        if (digitalRead(ENCODER_PIN_1) == LOW && encoderHalfRight == true) { // -->

            int millisSinceLastChange = millis() - encoderLastChange;
            if (millisSinceLastChange < 50) {
                encoderAccelerated = true;
            } else {
                encoderAccelerated = false;
            }
            encoderLastChange = millis();
            encoderHalfRight = false; // One whole click clockwise
            encoderState = NEXT;
        }
    }

    #if defined ESP32
    portEXIT_CRITICAL(&mux);
    #endif
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
    drawWifiSybol(260, 4, false);
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

void drawWifiSybol(int x, int y, bool connected) {
    byte pixelSize = 2;
    int color = ILI9341_RED;
    if (connected) {
        color = ILI9341_GREEN;
    }
    // first row
    tft.fillRect(x + (2 * pixelSize), y, 5 * pixelSize, pixelSize, color);
    // second row
    tft.fillRect(x + (1 * pixelSize), y + 1 * pixelSize, 1 * pixelSize, 1 * pixelSize, color);
    tft.fillRect(x + (7 * pixelSize), y + 1 * pixelSize, 1 * pixelSize, 1 * pixelSize, color);
    // third row
    tft.fillRect(x + (0 * pixelSize), y + 2 * pixelSize, 1 * pixelSize, 1 * pixelSize, color);
    tft.fillRect(x + (3 * pixelSize), y + 2 * pixelSize, 3 * pixelSize, 1 * pixelSize, color);
    tft.fillRect(x + (8 * pixelSize), y + 2 * pixelSize, 1 * pixelSize, 1 * pixelSize, color);
    // fourth row
    tft.fillRect(x + (2 * pixelSize), y + 3 * pixelSize, 1 * pixelSize, 1 * pixelSize, color);
    tft.fillRect(x + (6 * pixelSize), y + 3 * pixelSize, 1 * pixelSize, 1 * pixelSize, color);
    // fifth row
    tft.fillRect(x + (4 * pixelSize), y + 4 * pixelSize, 1 * pixelSize, 1 * pixelSize, color);
    // sixth row
    tft.fillRect(x + (3 * pixelSize), y + 5 * pixelSize, 3 * pixelSize, 1 * pixelSize, color);
    // seventh row
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
    drawFooterButtonsLabel("Next Trn", "Next Fnc", "Select");
}

int calculateXToCenter(String text, byte fontWidth) {
    int widthOfText = text.length() * fontWidth;
    int x = SCREEN_WIDTH / 2 - widthOfText / 2;
    return x;
}

void drawTurnoutView() {
    String title = "Turnouts";
    int titleX = calculateXToCenter(title, FONT_SIZE_2_WIDTH);
    tft.setCursor(titleX, MAIN_VIEW_TOP_Y);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);
    tft.print(title);

    drawBlockView(TURNOUT_ROWS, TURNOUT_COLUMNS, &turnoutList, selectedTurnout, previousTurnout, true);
}

void drawAccessoriesView() {
    String title = "Accessories";
    int titleX = calculateXToCenter(title, FONT_SIZE_2_WIDTH);
    tft.setCursor(titleX, MAIN_VIEW_TOP_Y);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);
    tft.print(title);

    drawBlockView(ACCESSORY_ROWS, ACCESSORY_COLUMNS, &accessoryList, selectedAccessory, previousAccessory, true);
}

void drawSensorsView() {
    String title = "Sensors";
    int titleX = calculateXToCenter(title, FONT_SIZE_2_WIDTH);
    tft.setCursor(titleX, MAIN_VIEW_TOP_Y);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);
    tft.print(title);

    drawBlockView(SENSOR_ROWS, SENSOR_COLUMNS, &sensorList, selectedSensor, previousSensor, true);
}

void drawPreferencesView() {
    String title = "Preferences";
    int titleX = calculateXToCenter(title, FONT_SIZE_2_WIDTH);
    tft.setCursor(titleX, MAIN_VIEW_TOP_Y);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);
    tft.println(title);

    //tft.println();
    //tft.println("IP Adrs: 192.168.1.150");
}

template <typename T>
void drawBlockView(uint8_t numberOfRows, uint8_t numberOfColumns, LinkedList<T> *list, byte selectedIndex, byte lastIndex, bool drawAll) {
    byte count = 0;
    uint8_t leftAndRightEdgePadding = 20; // sum of left and right padding
    uint8_t displayBoxHeight = 162; // available height for blocks
    uint8_t padding = 10; // horizontal and vertical padding between blocks
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

                    String textUpperLeft = list->get(count)->getName();
                    tft.setTextColor(ILI9341_WHITE);
                    tft.setCursor((thisBlockX + ((boxWidth / 2) - ((textUpperLeft.length() * FONT_SIZE_2_WIDTH) / 2))), thisBlockY + 4);
                    tft.print(textUpperLeft);

                    byte innerPaddingX = boxWidth / 8;
                    byte innerPaddingY = boxHeight / 4;
                    if (thisComponent->isActive()) {
                        tft.fillRect(thisBlockX + innerPaddingX, thisBlockY + FONT_SIZE_2_HEIGHT + 6, innerPaddingX * 6, innerPaddingY * 2, ILI9341_DARKGREEN);
                        String lOn = "On";
                        tft.setCursor((thisBlockX + ((boxWidth / 2) - ((lOn.length() * FONT_SIZE_2_WIDTH) / 2))), thisBlockY + FONT_SIZE_2_HEIGHT + 10);
                        tft.setTextColor(ILI9341_WHITE);
                        tft.print(lOn);
                    } else {
                        tft.fillRect(thisBlockX + innerPaddingX, thisBlockY + FONT_SIZE_2_HEIGHT + 6, innerPaddingX * 6, innerPaddingY * 2, ILI9341_DARKRED);
                        String lOff = "Off";
                        tft.setTextColor(ILI9341_WHITE);
                        tft.setCursor((thisBlockX + ((boxWidth / 2) - ((lOff.length() * FONT_SIZE_2_WIDTH) / 2))), thisBlockY + FONT_SIZE_2_HEIGHT + 10);
                        tft.print(lOff);
                    }
                }
            }
            count++;
        }
    }
}

void drawFooterButtonsLabel(String button1Label, String button2Label, String button3Label) {

    footerButtonsLabel[FOOTER_BUTTON_LEFT] = button1Label;
    footerButtonsLabel[FOOTER_BUTTON_MIDDLE] = button2Label;
    footerButtonsLabel[FOOTER_BUTTON_RIGHT] = button3Label;

    int footerYPosition = 219;
    int buttonWidth = 100;
    int buttonHeight = 21;
    int cornerRadius = 3;
    int widthOfLetter = 6;
    int paddingBetweenButtons = 10;

    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);

    if (footerButtonsLabel[FOOTER_BUTTON_LEFT] != "") {
        tft.fillRoundRect(0, footerYPosition, buttonWidth, buttonHeight, cornerRadius, FOOTER_BUTTON_COLOR);
        int xPosition = buttonWidth / 2 - (footerButtonsLabel[FOOTER_BUTTON_LEFT].length() * widthOfLetter);
        tft.setCursor(xPosition, footerYPosition + 3);
        tft.println(footerButtonsLabel[FOOTER_BUTTON_LEFT]);
    }

    if (footerButtonsLabel[FOOTER_BUTTON_MIDDLE] != "") {
        tft.fillRoundRect(110, footerYPosition, buttonWidth, buttonHeight, cornerRadius, FOOTER_BUTTON_COLOR);
        int xPosition = buttonWidth / 2 - (footerButtonsLabel[FOOTER_BUTTON_MIDDLE].length() * widthOfLetter);
        tft.setCursor(buttonWidth + paddingBetweenButtons + xPosition, footerYPosition + 3);
        tft.println(footerButtonsLabel[FOOTER_BUTTON_MIDDLE]);
    }

    if (footerButtonsLabel[FOOTER_BUTTON_RIGHT] != "") {
        tft.fillRoundRect(220, footerYPosition, buttonWidth, buttonHeight, cornerRadius, FOOTER_BUTTON_COLOR);
        int xPosition = buttonWidth / 2 - (footerButtonsLabel[FOOTER_BUTTON_RIGHT].length() * widthOfLetter);
        tft.setCursor((buttonWidth + paddingBetweenButtons) * 2 + xPosition, footerYPosition + 3);
        tft.println(footerButtonsLabel[FOOTER_BUTTON_RIGHT]);
    }
}

void footerButtonHighlighted(int button, bool isHighlighted) {
    String buttonLabel = footerButtonsLabel[button];
    if (buttonLabel != "") {
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
        int xPosition = buttonWidth / 2 - (buttonLabel.length() * widthOfLetter);
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

    tft.fillRect(12, trainNumberPositionTopLeftX + 2, 64, 22, ILI9341_BLACK); // fill in the background of the box so we can overwrite it
    tft.drawRect(10, trainNumberPositionTopLeftX, 68, 26, ILI9341_WHITE);
    tft.drawRect(11, trainNumberPositionTopLeftX + 1, 66, 24, ILI9341_WHITE);

    tft.setCursor(16, trainNumberPositionTopLeftX + 6);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);
    char trainNumberValue[6] = "";
    sprintf(trainNumberValue, "%05d", trainNumber);
    tft.println(trainNumberValue);
}

void drawTrainSpeed(int speed, byte direction) {
    tft.setCursor(85, MAIN_VIEW_TOP_Y + 6);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);
    tft.print("Speed:");
    char output[4] = "";
    sprintf(output, "%03d", speed);
    //if (output[0] != previousSpeed[0]) {
    tft.fillRect(85 + (6 * FONT_SIZE_2_WIDTH), MAIN_VIEW_TOP_Y + 6, FONT_SIZE_2_WIDTH, FONT_SIZE_2_HEIGHT, ILI9341_BLACK);
    tft.setCursor(85 + (6 * FONT_SIZE_2_WIDTH), MAIN_VIEW_TOP_Y + 6);
    tft.print(output[0]);
    //}
    //if (output[1] != previousSpeed[1]) {
    tft.fillRect(85 + (7 * FONT_SIZE_2_WIDTH), MAIN_VIEW_TOP_Y + 6, FONT_SIZE_2_WIDTH, FONT_SIZE_2_HEIGHT, ILI9341_BLACK);
    tft.setCursor(85 + (7 * FONT_SIZE_2_WIDTH), MAIN_VIEW_TOP_Y + 6);
    tft.print(output[1]);
    //}
    //if (output[2] != previousSpeed[2]) {
    tft.fillRect(85 + (8 * FONT_SIZE_2_WIDTH), MAIN_VIEW_TOP_Y + 6, FONT_SIZE_2_WIDTH, FONT_SIZE_2_HEIGHT, ILI9341_BLACK);
    tft.setCursor(85 + (8 * FONT_SIZE_2_WIDTH), MAIN_VIEW_TOP_Y + 6);
    tft.print(output[2]);
    //}
    sprintf(previousSpeed, "%03d", speed);

    ////
    int xIncrement = 7;
    int speedGraphValue = speed / 21;
    //tft.fillRect(80 + (10 * FONT_SIZE_2_WIDTH), MAIN_VIEW_TOP_Y + 6, xIncrement * 7, 16, ILI9341_BLACK);

    //if(direction == TRAIN_FORWARD) {
    tft.fillRect(80 + (10 * FONT_SIZE_2_WIDTH) + (xIncrement * 0), MAIN_VIEW_TOP_Y + 6 + 12, 5, 2, (speed > 0 ? ILI9341_GREEN : ILI9341_DARKGREY));
    tft.fillRect(80 + (10 * FONT_SIZE_2_WIDTH) + (xIncrement * 1), MAIN_VIEW_TOP_Y + 6 + 10, 5, 4, (speedGraphValue >= 1 ? ILI9341_GREEN : ILI9341_DARKGREY));
    tft.fillRect(80 + (10 * FONT_SIZE_2_WIDTH) + (xIncrement * 2), MAIN_VIEW_TOP_Y + 6 + 8, 5, 6, (speedGraphValue >= 2 ? ILI9341_GREEN : ILI9341_DARKGREY));
    tft.fillRect(80 + (10 * FONT_SIZE_2_WIDTH) + (xIncrement * 3), MAIN_VIEW_TOP_Y + 6 + 6, 5, 8, (speedGraphValue >= 3 ? ILI9341_GREEN : ILI9341_DARKGREY));
    tft.fillRect(80 + (10 * FONT_SIZE_2_WIDTH) + (xIncrement * 4), MAIN_VIEW_TOP_Y + 6 + 4, 5, 10, (speedGraphValue >= 4 ? ILI9341_GREEN : ILI9341_DARKGREY));
    tft.fillRect(80 + (10 * FONT_SIZE_2_WIDTH) + (xIncrement * 5), MAIN_VIEW_TOP_Y + 6 + 2, 5, 12, (speedGraphValue >= 5 ? ILI9341_GREEN : ILI9341_DARKGREY));
    tft.fillRect(80 + (10 * FONT_SIZE_2_WIDTH) + (xIncrement * 6), MAIN_VIEW_TOP_Y + 6 + 0, 5, 14, (speedGraphValue == 6 ? ILI9341_GREEN : ILI9341_DARKGREY));
    /*} else {
        tft.fillRect(80 + (10 * FONT_SIZE_2_WIDTH) + (xIncrement*6), MAIN_VIEW_TOP_Y + 6 + 12, 5,  2, (speed           >  0 ? ILI9341_RED : ILI9341_DARKGREY));
        tft.fillRect(80 + (10 * FONT_SIZE_2_WIDTH) + (xIncrement*5), MAIN_VIEW_TOP_Y + 6 + 10, 5,  4, (speedGraphValue >= 1 ? ILI9341_RED : ILI9341_DARKGREY));
        tft.fillRect(80 + (10 * FONT_SIZE_2_WIDTH) + (xIncrement*4), MAIN_VIEW_TOP_Y + 6 + 8,  5,  6, (speedGraphValue >= 2 ? ILI9341_RED : ILI9341_DARKGREY));
        tft.fillRect(80 + (10 * FONT_SIZE_2_WIDTH) + (xIncrement*3), MAIN_VIEW_TOP_Y + 6 + 6,  5,  8, (speedGraphValue >= 3 ? ILI9341_RED : ILI9341_DARKGREY));
        tft.fillRect(80 + (10 * FONT_SIZE_2_WIDTH) + (xIncrement*2), MAIN_VIEW_TOP_Y + 6 + 4,  5, 10, (speedGraphValue >= 4 ? ILI9341_RED : ILI9341_DARKGREY));
        tft.fillRect(80 + (10 * FONT_SIZE_2_WIDTH) + (xIncrement*1), MAIN_VIEW_TOP_Y + 6 + 2,  5, 12, (speedGraphValue >= 5 ? ILI9341_RED : ILI9341_DARKGREY));
        tft.fillRect(80 + (10 * FONT_SIZE_2_WIDTH) + (xIncrement*0), MAIN_VIEW_TOP_Y + 6 + 0,  5, 14, (speedGraphValue == 6 ? ILI9341_RED : ILI9341_DARKGREY));
       }
     */
}

void drawTrainDirection(byte direction) {
    tft.fillRect(260, MAIN_VIEW_TOP_Y + 6, 36, 16, ILI9341_BLACK);
    tft.setCursor(260, MAIN_VIEW_TOP_Y + 6);
    tft.setTextSize(2);
    if (direction == TRAIN_FORWARD) {
        tft.setTextColor(ILI9341_DARKGREEN);
        tft.print("FWD");
    } else {
        tft.setTextColor(ILI9341_DARKRED);
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

/*
   void editTrainNumber(int trainNumber, int indexToEdit) {
    int trainNumberPositionTopLeftX = 60;

    tft.fillRect(16 + (indexToEdit * 12) - 1, trainNumberPositionTopLeftX + 5, 10+2, 16, ILI9341_LIGHTGREY);         // fill in the background of the box so we can overwrite it

    tft.setCursor(16 + (indexToEdit * 12), trainNumberPositionTopLeftX + 6);
    tft.setTextColor(ILI9341_BLACK);
    tft.setTextSize(2);
    tft.println("9");
    char trainNumberValue[5] = "";
    sprintf(trainNumberValue, "%04d", trainNumber);
    tft.println(trainNumberValue);
   }*/
/*
   sendTrainCommand(trains[0]);
   trains[0].speed = 100;
   trains[0].direction = TRAIN_REVERSE;
   sendTrainCommand(trains[0]);

   editTrainNumber(trains[0].address, 0);
   delay(1000);
   drawTrainNumber(trains[0].address);
   delay(1000);

   editTrainNumber(trains[0].address, 1);
   delay(1000);
   drawTrainNumber(trains[0].address);
   delay(1000);

   editTrainNumber(trains[0].address, 2);
   delay(1000);
   drawTrainNumber(trains[0].address);
   delay(1000);

   editTrainNumber(trains[0].address, 3);
   delay(1000);
   drawTrainNumber(trains[0].address);
   delay(1000);*/

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
    tft.fillRect(FONT_SIZE_2_WIDTH * 13, 5, FONT_SIZE_2_WIDTH * 4, FONT_SIZE_2_HEIGHT, ILI9341_BLACK);
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(FONT_SIZE_2_WIDTH * 13, 5);
    tft.setTextSize(2);
    char amps[5] = "";
    sprintf(amps, "%d.%02d", (int)trackCurrent, (int)(trackCurrent * 100) % 100);
    tft.println(amps);
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
    Serial.println("toggling track power");
    trackPower = !trackPower;
}

void sendTrackPowerCommand() {
    if (trackPower == TRACK_POWER_ON) {
        readResponse(comm->send("<1>"));
    } else {
        String stopCommand = "";
        for (byte i = 0; i < trainList.size(); i++) {
            trainList.get(i)->setSpeed(0);
            stopCommand += trainList.get(i)->getEmergencyStopCommand();
        }
        stopCommand += "<0>";
        readResponse(comm->send(stopCommand));
    }
}

void sendTrainCommand(Train *train) {
    readResponse(comm->send(train->getSpeedCommand()));
}

void sendCurrentRequestCommand() {
    readResponse(comm->send("<c>"));
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
        char *commandTrimmed = &command[1];
        trackCurrent = atof(commandTrimmed) / 100;

        if (displayState != START) {
            drawTrackCurrent();
        }
        break;


    }
}

void parseTCommand(char *command) {
    int trainRegister = 0;
    int speed = 0;
    int direction = 0;
    Train *train;

    sscanf(&command[1], "%d %d %d", &trainRegister, &speed, &direction);
    train = getTrainByRegister(trainRegister);
    train->setSpeed(direction ? speed : speed * -1);
    train->setDirection(direction);

    drawTrainSpeed(train->getSpeed(), train->getDirection());
    drawTrainDirection(train->getDirection());
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

Train *getTrainByRegister(byte trainRegister) {
    for (int i = 0; i < trainList.size(); i++) {
        if (trainList.get(i)->getRegisterNumber() == trainRegister) {
            return trainList.get(i);
        }
    }
    return NULL;
}
