
#include <Servo.h>



Servo leverServo;
Servo knockerServo;
#define KNOCKER_IN 11
#define ARM_IN 3
const int leverservoPin = 5; //SQUARE
const int knockerservoPin = 6; //X

int Langle = 8;
int Kangle = 80;

void setup() {
  pinMode(KNOCKER_IN, INPUT);
  pinMode(ARM_IN, INPUT);
  Serial.begin(113200);
  leverServo.attach(leverservoPin);
  knockerServo.attach(knockerservoPin);
  leverServo.write(Langle);  // Move to 90° at startup
  delay(2000);
  knockerServo.write(Kangle);
  
}

void moveServoSmoothly(int angle) {
  if (angle == 8) {
    for (int pos = angle; pos <= 80; pos += 2) {
      leverServo.write(pos);
      delay(10);  // Small delay for smooth movement
    }
  } else {
    for (int pos = angle; pos >= 8; pos -= 1) {
      leverServo.write(pos);
      delay(10);  // Small delay for smooth movement
    }
    }
  }

void loop() {
  int knockerState = digitalRead(KNOCKER_IN);
  int armState = digitalRead(ARM_IN);
  Serial.print("Arm State: ");
  Serial.println(armState);
  // delay(500);
  if (armState == HIGH)
  {
    if (Langle == 8) {
      Langle = 90;
      // delay(1000);
    } else {
      Langle = 8;
      // delay(1000);
    }
    moveServoSmoothly(Langle);
  }

  Serial.print("Knocker State: ");
  Serial.println(knockerState);
  //  delay(500);
  if (knockerState == HIGH)
  {
    if (Kangle == 80) {
      Kangle = 120;
       delay(50);
    } else {
      Kangle = 80;
       delay(50);
    }
    knockerServo.write(Kangle);
  }
}

