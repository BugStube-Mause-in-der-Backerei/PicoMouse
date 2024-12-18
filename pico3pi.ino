#include <OPT3101.h>
#include <Wire.h>
#include <Pololu3piPlus2040.h>
#include <Arduino.h>
#include <SPI.h>
#include <U8g2lib.h>
#include <vector>

U8G2_SH1106_128X64_NONAME_F_4W_SW_SPI u8g2(U8G2_R0, 2, 3, U8X8_PIN_NONE, 0, 1);

// Allowed deviation (in degrees) relative to target angle
#define DEVIATION_THRESHOLD 3

OPT3101 sensor;
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

struct Wall {
  int x1, y1, x2, y2;  // Start- und Endpunkte der Wand
  Wall(int startX, int startY, int endX, int endY)
    : x1(startX), y1(startY), x2(endX), y2(endY) {}
};

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
bool startingWall = false;
int defaultSpeed = 80;
int currentPos[] = { 4, 4 };
int endPos[] = { 3, 0 };
String endProgramm = "";

int labyrinth[5][5][4] = {
  { { -1, -1, -1, -1 }, { -1, -1, -1, -1 }, { -1, -1, -1, -1 }, { -1, -1, -1, -1 }, { -1, -1, -1, -1 } },
  { { -1, -1, -1, -1 }, { -1, -1, -1, -1 }, { -1, -1, -1, -1 }, { -1, -1, -1, -1 }, { -1, -1, -1, -1 } },
  { { -1, -1, -1, -1 }, { -1, -1, -1, -1 }, { -1, -1, -1, -1 }, { -1, -1, -1, -1 }, { -1, -1, -1, -1 } },
  { { -1, -1, -1, -1 }, { -1, -1, -1, -1 }, { -1, -1, -1, -1 }, { -1, -1, -1, -1 }, { -1, -1, -1, -1 } },
  { { -1, -1, -1, -1 }, { -1, -1, -1, -1 }, { -1, -1, -1, -1 }, { -1, -1, -1, -1 }, { -1, -1, -1, -1 } },
};

bool labyrinthTracker[5][5] = {
  { false, false, false, false, false },
  { false, false, false, false, false },
  { false, false, false, false, false },
  { false, false, false, false, false },
  { false, false, false, false, false },
};

// method call to be able to define default parameters
void moveForward(bool forward = true, int count = 1);
void turn(char dir, int count = 1);


void setup() {
  u8g2.begin();
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

  if (buttonC.isPressed()) {
    endProgramm = "backtracking";
    delay(2000);

    doBacktracking();
  }
  
  if (buttonA.isPressed()) {
    endProgramm = "erkunden";
    delay(2000);

    doBacktracking();
  }
}

bool evaluateEnd() {
  if (endProgramm == "backtracking") {
    buzzer.play("f32");
    return (currentPos[0] == endPos[0] && currentPos[1] == endPos[1]);
  }
  if (endProgramm == "erkunden") {
    buzzer.play("f32");
    return allFieldsFilled(labyrinthTracker);
  }
}


void analyzeMaze(int maze[5][5][4], int width, int height, std::vector<Wall> &walls) {
  const int cellSize = 11;  // Größe jedes Kästchens in Pixel

  // Horizontale Wände (Norden und Süden)
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      // Nordwand
      if (maze[y][x][0] == 0) {
        walls.push_back(Wall(x * cellSize, y * cellSize, (x + 1) * cellSize, y * cellSize));
      }
      // Südwand
      if (maze[y][x][2] == 0) {
        walls.push_back(Wall(x * cellSize, (y + 1) * cellSize, (x + 1) * cellSize, (y + 1) * cellSize));
      }
    }
  }

  // Vertikale Wände (Westen und Osten)
  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      // Westwand
      if (maze[y][x][3] == 0) {
        walls.push_back(Wall(x * cellSize, y * cellSize, x * cellSize, (y + 1) * cellSize));
      }
      // Ostwand
      if (maze[y][x][1] == 0) {
        walls.push_back(Wall((x + 1) * cellSize, y * cellSize, (x + 1) * cellSize, (y + 1) * cellSize));
      }
    }
  }
}

String getDirection() {
  int32_t angle = (((int32_t)turnAngle >> 16) * 360) >> 16;
  String dir;
  if (angle < 45 && angle > -45) {
    dir = "north";
  } else if (angle > -130 && angle < -45) {
    dir = "east";
  } else if (angle < -130 || angle > 130) {
    dir = "south";
  } else if (angle < 130 && angle > 45) {
    dir = "west";
  }
  return dir;
}

void addWalls(int possibleWays[]) {
  turnSensorUpdate();
  String cur_direction = getDirection();
  // {N, O, S, W}
  if (cur_direction == "north") {
    labyrinth[currentPos[0]][currentPos[1]][0] = -possibleWays[1];
    labyrinth[currentPos[0]][currentPos[1]][1] = -possibleWays[2];
    // keine hintere süd wand, deswegen labyrinth[currentPos[0]][currentPos[1]][2] irrelevant
    labyrinth[currentPos[0]][currentPos[1]][3] = -possibleWays[0];
  }
  if (cur_direction == "east") {
    labyrinth[currentPos[0]][currentPos[1]][0] = -possibleWays[0];
    labyrinth[currentPos[0]][currentPos[1]][1] = -possibleWays[1];
    labyrinth[currentPos[0]][currentPos[1]][2] = -possibleWays[2];
    // keine hintere west wand, deswegen labyrinth[currentPos[0]][currentPos[1]][3] irrelevant
  }
  if (cur_direction == "south") {
    // keine hintere nord wand, deswegen labyrinth[currentPos[0]][currentPos[1]][0] irrelevant
    labyrinth[currentPos[0]][currentPos[1]][1] = -possibleWays[0];
    labyrinth[currentPos[0]][currentPos[1]][2] = -possibleWays[1];
    labyrinth[currentPos[0]][currentPos[1]][3] = -possibleWays[2];
  }
  if (cur_direction == "west") {
    labyrinth[currentPos[0]][currentPos[1]][0] = -possibleWays[2];
    // keine hintere ost wand, deswegen labyrinth[currentPos[0]][currentPos[1]][1] irrelevant
    labyrinth[currentPos[0]][currentPos[1]][2] = -possibleWays[0];
    labyrinth[currentPos[0]][currentPos[1]][3] = -possibleWays[1];
  }
  labyrinthTracker[currentPos[0]][currentPos[1]] = true;
}

void showWalls() {
  u8g2.firstPage();
  do {

    int width = 5;
    int height = 5;

    // Liste der Wände
    std::vector<Wall> walls;

    // Labyrinth analysieren
    analyzeMaze(labyrinth, width, height, walls);

    // Ergebnis ausgeben (z. B. über die serielle Konsole)
    Serial.println("blubb");
    for (const Wall &wall : walls) {
      u8g2.drawLine(wall.x1, wall.y1, wall.x2, wall.y2);
      Serial.print("Wall: (");
      Serial.print(wall.x1);
      Serial.print(", ");
      Serial.print(wall.y1);
      Serial.print(") to (");
      Serial.print(wall.x2);
      Serial.print(", ");
      Serial.print(wall.y2);
      Serial.println(")");
    }
    delay(100);
  } while (u8g2.nextPage());
}

void turn(char dir, int count) {
  int turnSpeed = 80;
  int speed = 0;
  if (dir == 'l') {
    motors.setSpeeds(-turnSpeed, turnSpeed);
    headingDirection += 90 * count;
  } else if (dir == 'r') {
    motors.setSpeeds(turnSpeed, -turnSpeed);
    headingDirection -= 90 * count;
  }

  if (headingDirection < 0) {
    headingDirection += 360;
  } else if (headingDirection >= 360) {
    headingDirection -= 360;
  }

  while (true) {
    turnSensorUpdate();
    int32_t angle = getCurrentAngle();

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

bool allFieldsFilled(bool labyrinth[5][5]) {
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 5; j++) {
      if (labyrinth[i][j] == false) {
        return false;  // Ein Feld ist -1
      }
    }
  }
  return true;  // Alle Felder sind != -1
}

void moveForward(bool forward, int count) {
  int speed = defaultSpeed;
  int speedLeft = 0;
  int speedRight = 0;
  int distanceFront = 80;
  int distanceSide = 200;
  bool hasSampled = false;
  float driveDistance = 17.25F * count;
  wallRight = true;
  wallLeft = true;
  sensor.setChannel(1);
  sensor.startSample();
  while (!sensor.isSampleDone()) {}
  sensor.readOutputRegs();

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
    if (sensor.isSampleDone() && Sr >= 2) {
      sensor.readOutputRegs();
      int distanceMM = sensor.distanceMillimeters;
      // recognize wall openings during moving
      switch (sensor.channelUsed) {
        case 0:
          if (wallLeft && distanceMM > distanceSide) {
            wallLeft = false;
          }
          if (distanceMM < 150) {
            speedLeft = 10;
          } else {
            speedLeft = 0;
          }
          break;
        case 1:
          if (distanceMM < distanceFront) {
            goto bailout;
          }
          break;
        case 2:
          if (wallRight && distanceMM > distanceSide) {
            wallRight = false;
          }
          if (distanceMM < 150) {
            speedRight = 10;
          } else {
            speedRight = 0;
          }
          break;
      }
      sensor.startSample();
    }


    countsRight += encoders.getCountsAndResetRight();
    // approximate the driven distance
    Sr += ((countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENZCE);

    int diff = headingDirection - getCurrentAngle();
    int absDiff = abs(diff);
    // diff shouldn't be negative; edge case: headingDirection = 0 and angle = 350 should be diff 10 and not 350
    if (absDiff > 100) {
      absDiff = 360 - absDiff;
    }

    // turnSpeed shouldn't be greater than 10 as the robot would start to wiggle
    int turnSpeed = speed * absDiff / 20;
    if (turnSpeed > 10) {
      turnSpeed = 10;
    } else if (speedLeft != 0 || speedRight != 0) {
      turnSpeed = 0;
    }

    if (Sr <= driveDistance && Sr >= -driveDistance) {
      if (Sr > driveDistance - 5 || Sr < -driveDistance + 5) {
        speed = defaultSpeed - (abs(Sr) * 3);
      }
      if (speed < 25) {
        speed = 25;
      }

      if (diff > 0 || diff < -270) {
        forward ? motors.setSpeeds(speed + speedLeft, speed + speedRight + turnSpeed) : motors.setSpeeds(-speed - turnSpeed, -speed);
      } else if (diff < 0) {
        forward ? motors.setSpeeds(speed + speedLeft + turnSpeed, speed + speedRight) : motors.setSpeeds(-speed, -speed - turnSpeed);
      } else {
        forward ? motors.setSpeeds(speed + speedLeft, speed + speedRight) : motors.setSpeeds(-speed, -speed);
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

  bool programmEnded = evaluateEnd();
  // end when on endPos
  if (programmEnded) {
    //get possible ways
    sensor.setChannel(1);
    for (int i = 0; i < 5; i++) {
      sensor.startSample();

      while (!sensor.isSampleDone()) {}
      sensor.readOutputRegs();

      if (sensor.distanceMillimeters > 170) {
        possibleWays[1] = 1;
      }
    }

    if (!wallLeft) {
      possibleWays[0] = 1;
    }
    if (!wallRight) {
      possibleWays[2] = 1;
    }
    addWalls(possibleWays);
    showWalls();
    return 1;
  }

  //get possible ways
  sensor.setChannel(1);
  for (int i = 0; i < 5; i++) {
    sensor.startSample();

    while (!sensor.isSampleDone()) {}
    sensor.readOutputRegs();

    if (sensor.distanceMillimeters > 170) {
      possibleWays[1] = 1;
    }
  }

  if (!wallLeft) {
    possibleWays[0] = 1;
  }
  if (!wallRight) {
    possibleWays[2] = 1;
  }

  if (startingWall == false) {
    labyrinth[currentPos[0]][currentPos[1]][2] = 0;
    startingWall = true;
  }
  addWalls(possibleWays);

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

      updateCurrentPos(false);
      switch (i) {
        case 0:
          moveForward(false);
          turn('r');
          break;
        case 1:
          moveForward(false);
          break;
        case 2:
          moveForward(false);
          turn('l');
          break;
      }
    }
  }
  return 0;
}


void updateCurrentPos(bool movedForward) {
  switch (headingDirection) {
    case 0:
      movedForward ? changeCurrentPos(0, -1) : changeCurrentPos(0, 1);
      break;
    case 90:
      movedForward ? changeCurrentPos(1, -1) : changeCurrentPos(1, 1);
      break;
    case 180:
      movedForward ? changeCurrentPos(0, 1) : changeCurrentPos(0, -1);
      break;
    case 270:
      movedForward ? changeCurrentPos(1, 1) : changeCurrentPos(1, -1);
      break;
  }
}

void changeCurrentPos(int field, int change) {
  currentPos[field] = currentPos[field] + change;
  Serial.println(currentPos[field]);
}

int32_t getCurrentAngle() {
  int32_t angle = (((int32_t)turnAngle >> 16) * 360) >> 16;
  if (angle < 0) {
    angle += 360;
  }

  return angle;
}
