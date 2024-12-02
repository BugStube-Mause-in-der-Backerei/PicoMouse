#include <OPT3101.h>
#include <Pololu3piPlus2040.h>
#include <Wire.h>

// Allowed deviation (in degrees) relative to target angle
#define DEVIATION_THRESHOLD 3

OPT3101 sensor;
OLED display;
Buzzer buzzer;
ButtonA buttonA;
ButtonB buttonB;
ButtonC buttonC;
LineSensors lineSensors;
BumpSensors bumpSensors;
Motors motors;
Encoders encoders;
RGBLEDs leds;
IMU imu;

#include "TurnSensor.h"

long countsRight = 0;
long prevRight = 0;
const int CLICKS_PER_ROTATION = 12;
const float GEAR_RATIO = 29.86F;
const int WHEEL_CIRCUMFERENZCE = 10.0531;
float Sr = 0.0F;
int headingDirection = 0;
bool wallLeft = true;
bool wallRight = true;
int defaultSpeed = 80;
int currentPos[] = { 1, 1 };
int endPos[] = { 3, 2 };


// String inputs[] = { "forward", "turn_right", "forward", "turn_right", "forward", "turn_left", "forward", "turn_left", "forward", "forward", "forward", "turn_left", "forward", "forward", "turn_right", "forward", "turn_right", "forward", "forward", "forward", "forward", "turn_right", "forward", "turn_right", "forward", "turn_left", "forward", "turn_left", "forward" };
String inputs[] = { "forward", "turn_right", "forward", "turn_right", "forward", "turn_left", "forward", "turn_left", "forward", "forward", "forward", "turn_left", "forward", "forward", "turn_right", "forward", "turn_right", "forward", "forward", "forward", "forward", "turn_right", "forward", "turn_right", "forward", "turn_left", "forward", "turn_left", "forward", "forward", "forward", "turn_left", "forward", "turn_right", "forward", "turn_left", "forward", "turn_right", "forward", "forward", "forward" };
int inputSize = sizeof(inputs) / sizeof(String);

// String inputsC[] = { "forward", "turn_right", "forward", "turn_right", "forward", "turn_right", "forward", "turn_right" };
String inputsC[] = { "forward", "turn_right", "forward", "turn_right", "forward", "turn_left" };
int inputSizeC = sizeof(inputsC) / sizeof(String);


// method call to be able to define default parameters
void moveForward(bool forward = true);


void setup() {
  Serial.begin(9600);
  Wire.begin();
  delay(1000);

  turnSensorSetup();
  turnSensorReset();

  sensor.init();
  if (sensor.getLastError()) {
    Serial.print(F("Failed to initialize OPT3101: error "));
    Serial.println(sensor.getLastError());
  }
  sensor.setFrameTiming(256);
  sensor.setBrightness(OPT3101Brightness::Adaptive);

  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();

  buzzer.play("C32");
}


void loop() {
  motors.setSpeeds(0, 0);
  bumpSensors.read();
  if (buttonA.isPressed()) {
    delay(2000);
    for (int i = 0; i < inputSize; i++) {
      handleInput(inputs[i]);
      motors.setSpeeds(0, 0);
      delay(100);
    }
  }

  if (buttonC.isPressed()) {
    delay(2000);

    // for (int i = 0; i < inputSizeC; i++) {
    //   handleInput(inputsC[i]);
    //   motors.setSpeeds(0, 0);
    //   delay(500);
    // }

    doBacktracking();
    displayPosition();

    // turnResist();
  }
}

void handleInput(String input) {
  if (input.equalsIgnoreCase("forward")) {
    Serial.println("moveForward");
    moveForward();
  } else if (input.equalsIgnoreCase("turn_left")) {
    Serial.println("turn_left");
    turn('l');
  } else if (input.equalsIgnoreCase("turn_right")) {
    Serial.println("turn_right");
    turn('r');
  }
}


void turn(char dir) {
  int turnSpeed = 80;
  int speed = 0;
  if (dir == 'l') {
    motors.setSpeeds(-turnSpeed, turnSpeed);
    headingDirection += 90;
  } else if (dir == 'r') {
    motors.setSpeeds(turnSpeed, -turnSpeed);
    headingDirection -= 90;
  }

  if (headingDirection < 0) {
    headingDirection += 360;
  } else if (headingDirection >= 360) {
    headingDirection -= 360;
  }

  while (true) {
    turnSensorUpdate();
    int32_t angle = (((int32_t)turnAngle >> 16) * 360) >> 16;
    if (angle < 0) {
      angle += 360;
    }

    int diff = headingDirection - angle;
    int absDiff = abs(diff);
    if (absDiff > 100) {
      absDiff = 360 - absDiff;
    }
    speed = turnSpeed * absDiff / 180;
    speed += 25;

    if (dir == 'l') {
      motors.setSpeeds(-speed, speed);
    } else if (dir == 'r') {
      motors.setSpeeds(speed, -speed);
    }

    int threshPos = headingDirection + DEVIATION_THRESHOLD;
    if (threshPos < 0) {
      threshPos += 360;
    }

    int threshNeg = headingDirection - DEVIATION_THRESHOLD;
    if (threshPos < 0) {
      threshPos += 360;
    }

    if (angle <= threshPos && angle >= threshNeg) {
      motors.setSpeeds(0, 0);
      break;
    }
  }
}


void moveForward(bool forward) {
  int speed = defaultSpeed;
  bool hasSampled = false;
  int distanceFront = 80;
  int distanceSide = 200;
  wallRight = true;
  wallLeft = true;
  sensor.setChannel(1);
  sensor.sample();
  if (forward && sensor.distanceMillimeters < distanceFront) {
    return;
  }

  for (int i = 0; i < 3; i++) {
    sensor.setChannel(i);
    sensor.startSample();
  }

  Sr = 0.0F;
  countsRight = encoders.getCountsAndResetRight();
  countsRight = 0;
  prevRight = 0;

  while (true) {
    turnSensorUpdate();

    sensor.nextChannel();
    // start sample evaluation shortly after moving to avoid detecting an opening at the start
    if (sensor.isSampleDone() && Sr > 2) {
      sensor.readOutputRegs();
      switch (sensor.channelUsed) {
        case 0:
          if (wallLeft && sensor.distanceMillimeters > distanceSide) {
            wallLeft = false;
          }
          break;
        case 1:
          if (sensor.distanceMillimeters < distanceFront) {
            goto bailout;
          }
          break;
        case 2:
          if (wallRight && sensor.distanceMillimeters > distanceSide) {
            wallRight = false;
          }
          break;
      }
      sensor.startSample();
    }

    countsRight += encoders.getCountsAndResetRight();

    Sr += ((countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENZCE);

    int32_t angle = (((int32_t)turnAngle >> 16) * 360) >> 16;
    if (angle < 0) {
      angle += 360;
    }

    int diff = headingDirection - angle;
    int absDiff = abs(diff);
    if (absDiff > 100) {
      absDiff = 360 - absDiff;
    }

    int turnSpeed = speed * absDiff / 20;
    if (turnSpeed > 10) {
      turnSpeed = 10;
    }

    if (Sr < 18 && Sr > -18) {
      if (Sr > 5 || Sr < -5) {
        speed = 80 - (abs(Sr) * 3);
      }
      if (speed < 25) {
        speed = 25;
      }
      if (diff > 0 || diff < -270) {
        forward ? motors.setSpeeds(speed, speed + turnSpeed) : motors.setSpeeds(-speed - turnSpeed, -speed);
      } else if (diff < 0) {
        forward ? motors.setSpeeds(speed + turnSpeed, speed) : motors.setSpeeds(-speed, -speed - turnSpeed);
      } else {
        forward ? motors.setSpeeds(speed, speed) : motors.setSpeeds(-speed, -speed);
      }
    } else {
      break;
    }

    prevRight = countsRight;
  }

bailout:
  motors.setSpeeds(0, 0);
}


int doBacktracking() {
  int possibleWays[] = { 0, 0, 0 };
  int possibleWaysSize = sizeof(possibleWays) / sizeof(int);

  // end when on endPos
  if (currentPos[0] == endPos[0] && currentPos[1] == endPos[1]) {
    return 1;
  }

  //get possible ways
  sensor.setChannel(1);
  sensor.sample();
  if (sensor.distanceMillimeters > 200) {
    possibleWays[1] = 1;
  }
  if (!wallLeft) {
    possibleWays[0] = 1;
  }
  if (!wallRight) {
    possibleWays[2] = 1;
  }

  display.clear();
  display.gotoXY(0, 0);
  display.print(currentPos[0]);
  display.print(" ");
  display.print(currentPos[1]);
  display.gotoXY(0, 1);
  display.print(possibleWays[0]);
  display.print(" ");
  display.print(possibleWays[1]);
  display.print(" ");
  display.print(possibleWays[2]);

  for (int i = 0; i < possibleWaysSize; i++) {
    if (possibleWays[i] == 1) {
      switch (i) {
        case 0:
          turn('l');
          moveForward();
          updateCurrentPos(true);
          break;
        case 1:
          moveForward();
          updateCurrentPos(true);
          break;
        case 2:
          turn('r');
          moveForward();
          updateCurrentPos(true);
          break;
      }

      int result = doBacktracking();
      if (result == 1) {
        return 1;
      }

      switch (i) {
        case 0:
          moveForward(false);
          turn('r');
          updateCurrentPos(false);
          break;
        case 1:
          moveForward(false);
          updateCurrentPos(false);
          break;
        case 2:
          moveForward(false);
          turn('l');
          updateCurrentPos(false);
          break;
      }

      display.clear();
      display.gotoXY(0, 0);
      display.print(currentPos[0]);
      display.print(" ");
      display.print(currentPos[1]);
      display.gotoXY(0, 1);
      display.print(possibleWays[0]);
      display.print(" ");
      display.print(possibleWays[1]);
      display.print(" ");
      display.print(possibleWays[2]);
    }
  }
  return 0;
}


void updateCurrentPos(bool movedForward) {
  switch (headingDirection) {
    case 0:
      movedForward ? changeCurrentPos(0, 1) : changeCurrentPos(0, -1);
      break;
    case 90:
      movedForward ? changeCurrentPos(1, -1) : changeCurrentPos(1, 1);
      break;
    case 180:
      movedForward ? changeCurrentPos(0, -1) : changeCurrentPos(0, 1);
      break;
    case 270:
      movedForward ? changeCurrentPos(1, 1) : changeCurrentPos(1, -1);
      break;
  }
}


void changeCurrentPos(int field, int change) {
  currentPos[field] = currentPos[field] + change;
}


void displayPosition() {
  display.clear();
  display.gotoXY(0, 0);
  display.print(currentPos[0]);
  display.print(" ");
  display.print(currentPos[1]);
}


void turnResist() {
  while (true) {
    turnSensorUpdate();

    int32_t turnSpeed = -(int32_t)turnAngle / (turnAngle1 / 28)
                        - turnRate / 40;

    display.clear();
    display.gotoXY(0, 0);
    display.print(turnSpeed);
    display.print(F("   "));
    motors.setSpeeds(-turnSpeed, turnSpeed);
  }
}
