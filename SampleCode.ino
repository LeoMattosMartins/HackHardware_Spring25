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
#include <WiFi.h>
// Load SR library
// https://github.com/Simsso/ShiftRegister74HC595
#include <ShiftRegister74HC595.h>

// Replace with your network credentials
const char* ssid = "hackhardware_leo";
const char* password = "spring25";

// Set web server port number to 80
WiFiServer server(80);

// Variable to store the HTTP request
String header;

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

#define MOVESPEED 200 // default speed for movements

ShiftRegister74HC595<1> SR(SR_DATA, SR_CLK, SR_LTCH); // Initialize Serial Register on Driver Board (Data, Clock, Latch)

uint8_t motor_state = 0x00; // bitstring that defines the behavior of each motor driver via the Serial Register

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

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

void setup() {
  Serial.begin(115200);
  // Sends a Hello World Debug message
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

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  //WiFi.begin(ssid, password);
  /**/
  WiFi.softAP("hackhardware_leo", "spring25"); // Set SSID and Password for AP mode
  Serial.println("Access Point Started");
  Serial.println("IP Address: ");
  Serial.println(WiFi.softAPIP());  // Display the IP address of the AP

  // int ii = 0;
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(500);
  //   Serial.print("--AWAITING CONNECTION [%d]--\n", ii++);
  // }
  
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
}

void loop(){
  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected() && currentTime - previousTime <= timeoutTime) {  // loop while the client's connected
      currentTime = millis();
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
// Move Left
            // turns the GPIOs on and off
            if (header.indexOf("GET /MoveLeftOn") >= 0) {
              // Prints that we're moving left
              Serial.println("MoveLeft");

              // Sets flags that we're moving left
              isMoveLeftOn = 1;
              isMoveRightOn = 0;
              isMoveForwardOn = 0;
              isMoveBackwardOn = 0;
              isRotateLeftOn = 0;
              isRotateRightOn = 0;
              
              // Turns on pins to move left
              MoveLeft();
// Move Right
            } else if (header.indexOf("GET /MoveRightOn") >= 0) {
              // Prints that we're moving right
              Serial.println("MoveRight");

              // Sets flags that we're moving right
              isMoveLeftOn = 0;
              isMoveRightOn = 1;
              isMoveForwardOn = 0;
              isMoveBackwardOn = 0;
              isRotateLeftOn = 0;
              isRotateRightOn = 0;

              // Turns on pins to move right
              MoveRight();
// Move Forward
            } else if (header.indexOf("GET /MoveForwardOn") >= 0) {
              // Prints that we're moving forward
              Serial.println("MoveForward");

              // Sets flags that we're moving forward
              isMoveLeftOn = 0;
              isMoveRightOn = 0;
              isMoveForwardOn = 1;
              isMoveBackwardOn = 0;
              isRotateLeftOn = 0;
              isRotateRightOn = 0;

              // Turns on pins to move forward
              MoveForward();
// Move Backward
            } else if (header.indexOf("GET /MoveBackwardOn") >= 0) {
              // Prints that we're moving backward
              Serial.println("MoveBackward");

              // Sets flags that we're moving backward
              isMoveLeftOn = 0;
              isMoveRightOn = 0;
              isMoveForwardOn = 0;
              isMoveBackwardOn = 1;
              isRotateLeftOn = 0;
              isRotateRightOn = 0;

              // Turns on pins to move backward
              MoveBackward();
// Rotate Left
            } else if (header.indexOf("GET /RotateLeftOn") >= 0) {
              // Prints that we're rotating left
              Serial.println("RotateLeft");

              // Sets flags that we're rotating left
              isMoveLeftOn = 0;
              isMoveRightOn = 0;
              isMoveForwardOn = 0;
              isMoveBackwardOn = 0;
              isRotateLeftOn = 1;
              isRotateRightOn = 0;

              // Turns on pins to rotate left
              RotateLeft();
// Rotate Right
            } else if (header.indexOf("GET /RotateRightOn") >= 0) {
              // Prints that we're rotating right
              Serial.println("RotateRight");

              // Sets flags that we're rotating right
              isMoveLeftOn = 0;
              isMoveRightOn = 0;
              isMoveForwardOn = 0;
              isMoveBackwardOn = 0;
              isRotateLeftOn = 0;
              isRotateRightOn = 1;

              // Turns on pins to rotate right
              RotateRight();
// Stationary
            } else{
              // Prints that the robot is stationary
              Serial.println("Stationary");

              // Set all other motion flags to 0
              isMoveLeftOn = 0;
              isMoveRightOn = 0;
              isMoveForwardOn = 0;
              isMoveBackwardOn = 0;
              isRotateLeftOn = 0;
              isRotateRightOn = 0;

              // Turns off all pins
              Stationary();

            }
            
            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons 
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #555555;}</style></head>");
            
            // Web Page Heading
            client.println("<body><h1>ESP32 Web Server</h1>");
            


            // Changes UI based on which button is pressed.

            // Display Move Left Button
            client.println("Move Left");
            // If the MoveLeft is off, it displays the MoveLeft button       
            if (!isMoveLeftOn) {
              client.println("<p><a href=\"/MoveLeftOn\"><button class=\"button\">MoveLeft</button></a></p>");
            } else {
              client.println("<p><a href=\"/Stationary\"><button class=\"button button2\">OFF</button></a></p>");
            } 

            // Display Move Right Button
            client.println("Move Right");
            // If the MoveRight is off, it displays the MoveRight button       
            if (!isMoveRightOn) {
              client.println("<p><a href=\"/MoveRightOn\"><button class=\"button\">MoveRight</button></a></p>");
            } else {
              client.println("<p><a href=\"/Stationary\"><button class=\"button button2\">OFF</button></a></p>");
            } 

            // Display Move Forward Button
            client.println("Move Forward");
            // If the MoveForward is off, it displays the MoveForward button       
            if (!isMoveForwardOn) {
              client.println("<p><a href=\"/MoveForwardOn\"><button class=\"button\">MoveForward</button></a></p>");
            } else {
              client.println("<p><a href=\"/Stationary\"><button class=\"button button2\">OFF</button></a></p>");
            } 

            // Display Move Backward Button
            client.println("Move Backward");
            // If the MoveBackward is off, it displays the MoveBackward button       
            if (!isMoveBackwardOn) {
              client.println("<p><a href=\"/MoveBackwardOn\"><button class=\"button\">MoveBackward</button></a></p>");
            } else {
              client.println("<p><a href=\"/Stationary\"><button class=\"button button2\">OFF</button></a></p>");
            } 

            // Display Rotate Left Button
            client.println("Rotate Left");
            // If the RotateLeft is off, it displays the RotateLeft button       
            if (!isRotateLeftOn) {
              client.println("<p><a href=\"/RotateLeftOn\"><button class=\"button\">RotateLeft</button></a></p>");
            } else {
              client.println("<p><a href=\"/Stationary\"><button class=\"button button2\">OFF</button></a></p>");
            } 

            // Display Rotate Right Button
            client.println("Rotate Right");
            // If the RotateRight is off, it displays the RotateRight button       
            if (!isRotateRightOn) {
              client.println("<p><a href=\"/RotateRightOn\"><button class=\"button\">RotateRight</button></a></p>");
            } else {
              client.println("<p><a href=\"/Stationary\"><button class=\"button button2\">OFF</button></a></p>");
            }
               

            client.println("</body></html>");
            
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}

/*
Copyright 2025 Dakota Winslow and Justin Nascimento

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/