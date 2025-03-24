// by Dakota Winslow and Justin Nascimento
// For RASTICxHachHardware Spring 2025

/*
This example firmware is to give you a starting point for your robot. 
It allows control via a simple web server. You should expand on this code
to improve the functionality of your robot, or simply ignore this and
write your own code from scratch.

The pin definitions below are for the QA009 ESP32 Max V1.0. This board 
conveniently has a pin layout that matches the Arduino Uno, allowwing the 
use of the L293D Motor Driver Shield. 

***IMPORTANT NOTE***: The ESP32 has a few pins that must not be forced
low/high during boot. Unfortunately, pin 12 is one of those pins, and it is 
mapped to pin 7 on the L293D Motor Driver Shield. Pin 7 is the enable pin of
the shift register on the shield, which is held high (off) by default. To
avoid boot issues, you must remove pin 7 from the shield and connect the the
pad to another unused pin. In this code, we use pin 4 as the new enable pin.
*/

// Load Wi-Fi library
// Load SR library
// https://github.com/Simsso/ShiftRegister74HC595
#include <ShiftRegister74HC595.h>
#include <Bluepad32.h>

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// Replace with your network credentials
const char* ssid = "hackhardware_leo";
const char* password = "spring25";

// Set web server port number to 80

// Variable to store the HTTP request

// Pin Definitions
#define LED_BUILTIN 2

#define M1 17 //M1 PWM pin, Driver Board pin 11
#define M2 26 //M2 PWM pin, Driver Board pin 3
#define M3 13 //M3 PWM pin, Driver Board pin 6
#define M4 14 //M4 PWM pin, Driver Board pin 5

#define M1_REV false // set to true to reverse M1 direction
#define M2_REV true // set to true to reverse M2 direction
#define M3_REV false // set to true to reverse M3 direction
#define M4_REV false // set to true to reverse M4 direction

#define S1 19 //S1 PWM pin, Driver Board pin 9
#define S2 18 //S2 PWM pin, Driver Board pin 10

#define SR_DATA 23 // Serial Register data pin, Driver Board pin 8
#define SR_CLK 25 // Serial Register clock pin, Driver Board pin 4
#define SR_LTCH 16 // Serial Register Latch Pin, Driver Board pin 12
#define SR_EN 4 // Serial Register Enable, Driver Board pin 7
// ^ the SR_EN pin is moved from pin 12 to pin 4 to avoid ESP32 boot issues

#define OFF 0b00  
#define FWD 0b01 
#define REV 0b10

#define MOVESPEED 170 // default speed for movements

ShiftRegister74HC595<1> SR(SR_DATA, SR_CLK, SR_LTCH); // Initialize Serial Register on Driver Board (Data, Clock, Latch)

uint8_t motor_state = 0x00; // bitstring that defines the behavior of each motor driver via the Serial Register



void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}


void setM1(uint8_t speed, uint8_t dir)
// sets the speed and direction of a motor, given by the pin number of that motor.
{
  if (M1_REV) dir = ~dir; // Swaps FWD and REV if motor turns the wrong way
  // Clear bits 2 and 3 in SR_state
  motor_state &= ~(0b00001100); // Mask to clear bits 2 and 3
  // Set bits 2 and 3 with the value of dir
  motor_state |= (dir << 2);    // Shift dir to align with bits 2 and 3, then OR
  // create temporary const for the SR
  const uint8_t tmp = (motor_state);
  // Update the SR
  SR.setAll(&tmp);
  // set the speed
  analogWrite(M1, speed);
}

void setM2(uint8_t speed, uint8_t dir)
// sets the speed and direction of a motor, given by the pin number of that motor.
{
  if (M2_REV) dir = ~dir; // Swaps FWD and REV if motor turns the wrong way
  // Clear bits 1 and 4 in SR_state
  motor_state &= ~(0b00010010); 
  // Set bit 1 
  motor_state |= (dir & 0b10);
  // Set bit 4
  motor_state |= (dir & 0b01) << 4; // Shift left 4
  // create temporary const for the SR
  const uint8_t tmp = (motor_state);
  // Update the SR
  SR.setAll(&tmp);
  // set the speed
  analogWrite(M2, speed);
}

void setM3(uint8_t speed, uint8_t dir)
// sets the speed and direction of a motor, given by the pin number of that motor.
{
  if (M3_REV) dir = ~dir; // Swaps FWD and REV if motor turns the wrong way
  // Clear bits 7 and 5 in SR_state
  motor_state &= ~(0b10100000);
  // Set bit 5
  motor_state |= (dir & 0b10) << 4;
  // Set bit 7
  motor_state |= (dir & 0b01) << 7; 
  // create temporary const for the SR
  const uint8_t tmp = (motor_state);
  // Update the SR
  SR.setAll(&tmp);
  // set the speed
  analogWrite(M3, speed);
}
void setM4(uint8_t speed, uint8_t dir)
// sets the speed and direction of a motor, given by the pin number of that motor.
{
  if (M4_REV) dir = ~dir; // Swaps FWD and REV if motor turns the wrong way
  // Clear bits 0 and 6 in SR_state
  motor_state &= ~(0b01000001);
  // Set bit 6
  motor_state |= (dir & 0b10) << 5;
  // Set bit 0
  motor_state |= (dir & 0b01); 
  // create temporary const for the SR
  const uint8_t tmp = (motor_state);
  // Update the SR
  SR.setAll(&tmp);
  // set the speed
  analogWrite(M4, speed);
}

// Variables to store the current output state
bool isMoveLeftOn = 0;
bool isMoveRightOn = 0;
bool isMoveForwardOn = 0;
bool isMoveBackwardOn = 0;
bool isRotateLeftOn = 0;
bool isRotateRightOn = 0;


// Motion Functions
void MoveLeft(){
  setM1(MOVESPEED, FWD);
  setM2(MOVESPEED, REV);
  setM3(MOVESPEED, REV);
  setM4(MOVESPEED, FWD);
  return;
}
void MoveRight(){
  setM1(MOVESPEED, REV);
  setM2(MOVESPEED, FWD);
  setM3(MOVESPEED, FWD);
  setM4(MOVESPEED, REV);
  return;
}
void MoveForward(){
  setM1(MOVESPEED, FWD);
  setM2(MOVESPEED, FWD);
  setM3(MOVESPEED, REV);
  setM4(MOVESPEED, REV);
  return;
}
void MoveBackward(){
  setM1(MOVESPEED, REV);
  setM2(MOVESPEED, REV);
  setM3(MOVESPEED, FWD);
  setM4(MOVESPEED, FWD);
  return;
}
void RotateLeft(){
  setM1(MOVESPEED, FWD);
  setM2(MOVESPEED, REV);
  setM3(MOVESPEED, FWD);
  setM4(MOVESPEED, REV);
  return;
}
void RotateRight(){
  setM1(MOVESPEED, REV);
  setM2(MOVESPEED, FWD);
  setM3(MOVESPEED, REV);
  setM4(MOVESPEED, FWD);
  return;
}
void Stationary(){
  setM1(0, OFF);
  setM2(0, OFF);
  setM3(0, OFF);
  setM4(0, OFF);
  return;
}

void dumpGamepad(ControllerPtr ctl) {
  // Read gamepad inputs
    int axisX = ctl->axisX();     // Left joystick X-axis
    int axisY = ctl->axisY();     // Left joystick Y-axis
    int axisRX = ctl->axisRX();   // Right joystick X-axis
    int brake = ctl->brake(); // brake button

    // Dead zone threshold to avoid unintentional movements
    int threshold = 150;

    // Determine movement based on joystick values
    if (axisX < -threshold) {  
        Serial.println("Move Left");
         isMoveLeftOn = 1;
          isMoveRightOn = 0;
          isMoveForwardOn = 0;
          isMoveBackwardOn = 0;
          isRotateLeftOn = 0;
          isRotateRightOn = 0;
        MoveLeft();
    } 
    else if (axisX > threshold) {  
        Serial.println("Move Right");
        isMoveLeftOn = 0;
        isMoveRightOn = 1;
        isMoveForwardOn = 0;
        isMoveBackwardOn = 0;
        isRotateLeftOn = 0;
        isRotateRightOn = 0;
        MoveRight();
    } 
    else if (axisY > threshold) {  
        Serial.println("Move Forward");
        isMoveLeftOn = 0;
        isMoveRightOn = 0;
        isMoveForwardOn = 1;
        isMoveBackwardOn = 0;
        isRotateLeftOn = 0;
        isRotateRightOn = 0;
        MoveForward();
    } 
    else if (axisY < -threshold) {  
        Serial.println("Move Backward");
         isMoveLeftOn = 0;
         isMoveRightOn = 0;
         isMoveForwardOn = 0;
         isMoveBackwardOn = 1;
        isRotateLeftOn = 0;
        isRotateRightOn = 0;
        MoveBackward();
    } 
    else if (axisRX > threshold) {  
        Serial.println("Rotate Right");
         isMoveLeftOn = 0;
              isMoveRightOn = 0;
              isMoveForwardOn = 0;
              isMoveBackwardOn = 0;
              isRotateLeftOn = 0;
              isRotateRightOn = 1;
        RotateRight();
    } 
    else if (axisRX < -threshold) {  
        Serial.println("Rotate Left");
           isMoveLeftOn = 0;
              isMoveRightOn = 0;
              isMoveForwardOn = 0;
              isMoveBackwardOn = 0;
              isRotateLeftOn = 1;
              isRotateRightOn = 0;
        RotateLeft();
    } 
    else if (brake > 20) {  // If throttle is pressed, stay stationary
        Serial.println("Stationary");
          isMoveLeftOn = 0;
              isMoveRightOn = 0;
              isMoveForwardOn = 0;
              isMoveBackwardOn = 0;
              isRotateLeftOn = 0;
              isRotateRightOn = 0;
        Stationary();
    }
}

void processGamepad(ControllerPtr ctl) {
    // There are different ways to query whether a button is pressed.
    // By query each button individually:
    //  a(), b(), x(), y(), l1(), etc...
    if (ctl->a()) {
        static int colorIdx = 0;
        // Some gamepads like DS4 and DualSense support changing the color LED.
        // It is possible to change it by calling:
        switch (colorIdx % 3) {
            case 0:
                // Red
                ctl->setColorLED(255, 0, 0);
                break;
            case 1:
                // Green
                ctl->setColorLED(0, 255, 0);
                break;
            case 2:
                // Blue
                ctl->setColorLED(0, 0, 255);
                break;
        }
        colorIdx++;
    }

    if (ctl->b()) {
        // Turn on the 4 LED. Each bit represents one LED.
        static int led = 0;
        led++;
        // Some gamepads like the DS3, DualSense, Nintendo Wii, Nintendo Switch
        // support changing the "Player LEDs": those 4 LEDs that usually indicate
        // the "gamepad seat".
        // It is possible to change them by calling:
        ctl->setPlayerLEDs(led & 0x0f);
    }

    if (ctl->x()) {
        // Some gamepads like DS3, DS4, DualSense, Switch, Xbox One S, Stadia support rumble.
        // It is possible to set it by calling:
        // Some controllers have two motors: "strong motor", "weak motor".
        // It is possible to control them independently.
        ctl->playDualRumble(0 /* delayedStartMs */, 250 /* durationMs */, 0x80 /* weakMagnitude */,
                            0x40 /* strongMagnitude */);
    }

    // Another way to query controller data is by getting the buttons() function.
    // See how the different "dump*" functions dump the Controller info.
    dumpGamepad(ctl);
}

void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            } else {
                Serial.println("Unsupported controller");
            }
        }
    }
}
// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

void setup() {
  Serial.begin(115200);
  // Sends a Hello World Debug message
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But it might also fix some connection / re-connection issues.
    BP32.forgetBluetoothKeys();

    // Enables mouse / touchpad support for gamepads that support them.
    // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
    // - First one: the gamepad
    // - Second one, which is a "virtual device", is a mouse.
    // By default, it is disabled.
    BP32.enableVirtualDevice(false);

  Serial.println("Starting RASTICxHackH Spring 2025 Controller...");
  
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(M3, OUTPUT);
  pinMode(M4, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(SR_DATA, OUTPUT);
  pinMode(SR_CLK, OUTPUT);
  pinMode(SR_LTCH, OUTPUT);
  pinMode(SR_EN, OUTPUT);

  const uint8_t tmp = 0;
  SR.setAll(&tmp); // Set all output pin of shift register to 0.

  digitalWrite(SR_EN, LOW); // Activate the Serial register (Active Low)

  

  

}

void loop(){
  bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();

    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    //     vTaskDelay(1);
    delay(150);
  
}

/*
Copyright 2025 Dakota Winslow and Justin Nascimento

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/