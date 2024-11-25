#include <OPT3101.h>
#include <Pololu3piPlus2040.h>
#include <Wire.h>

#define CALIBRATION_SAMPLES 79  // Number of compass readings to take when calibrating

// Allowed deviation (in degrees) relative to target angle that must be achieved before driving straight
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

uint16_t speedStraightLeft;  // Maximum motor speed when going straight; variable speed when turning
uint16_t speedStraightRight;
uint16_t turnBaseSpeed;  // Base speed when turning (added to variable speed)

IMU::vector<int16_t> m_max;  // maximum magnetometer values, used for calibration
IMU::vector<int16_t> m_min;  // minimum magnetometer values, used for calibration


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

  bumpSensors.calibrate();

  // calibrateTurnSensor();

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
    moveForward(false);
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
    if(angle < 0){
      angle += 360;
    }

    int diff = headingDirection - angle;
    int absDiff = abs(diff);
    if(absDiff > 100){
      absDiff = 360 - absDiff;
    }
    speed = turnSpeed * absDiff/180;
    speed += 25;

    if (dir == 'l') {
      motors.setSpeeds(-speed, speed);
    } else if (dir == 'r') {
      motors.setSpeeds(speed, -speed);
    }

    int threshPos = headingDirection + DEVIATION_THRESHOLD;
    if(threshPos<0){
      threshPos +=360;
    }

    int threshNeg = headingDirection - DEVIATION_THRESHOLD;
    if(threshPos<0){
      threshPos +=360;
    }

    
    
    display.clear();
    display.gotoXY(0, 0);
    display.print(angle);
    display.print(", ");
    display.print(headingDirection);
    if (angle <= threshPos && angle >= threshNeg) {
      motors.setSpeeds(0, 0);
      break;
    }
  }
}

void moveForward(bool forward) {
  sensor.setChannel(1);
  sensor.sample();
  if(sensor.distanceMillimeters < 80){
    return;
  }
  sensor.startSample();
  Sr = 0.0F;
  countsRight = encoders.getCountsAndResetRight();
  countsRight = 0;
  prevRight = 0;
  int speed = 80;
  while (true) {
    turnSensorUpdate();

    if (sensor.isSampleDone()) {
      sensor.readOutputRegs();
      if(sensor.distanceMillimeters < 80){
        break;
      }
      sensor.startSample();
    }

    countsRight += encoders.getCountsAndResetRight();

    Sr += ((countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENZCE);

    
    // int32_t turnSpeed = -(int32_t)turnAngle / (turnAngle1 / 28) - turnRate / 40;
    int32_t angle = (((int32_t)turnAngle >> 16) * 360) >> 16;
    if(angle < 0){
      angle += 360;
    }

    int diff = headingDirection - angle;
    int absDiff = abs(diff);
    if(absDiff > 100){
      absDiff = 360 - absDiff;
    }

    int turnSpeed = speed * absDiff/20;
    if (turnSpeed > 10) {
      turnSpeed = 10;
    }

    display.clear();
    display.gotoXY(0, 0);
    display.print(turnSpeed);
    display.print(",");
    display.print(absDiff);
    display.gotoXY(0, 1);
    display.print(diff);
    display.print(",");
    display.print(angle);


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
  motors.setSpeeds(0, 0);
}

int doBacktracking(){
  int possibleWays[3];
  int possibleWaysSize = sizeof(possibleWays) / sizeof(int);
  
  //get possible ways


  // TODO: some return function when maze is solved
  if(1==0){
    return 1;
  }

  for(int i = 0; i<possibleWaysSize; i++){
    if(possibleWays[i]){
      switch (i) {
      case 0:
        turn('l');
        moveForward();
        break;
      case 1:
        moveForward();
        break;
      case 2:
        turn('r');
        moveForward();
        break;
      }

      int result = doBacktracking();
      if(result == 1){
        return 1;
      }

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
}

// Converts x and y components of a vector to a heading in degrees.
// This calculation assumes that the 3pi+ is always level.
template<typename T> float heading(IMU::vector<T> v) {
  float x_scaled = 2.0 * (float)(v.x - m_min.x) / (m_max.x - m_min.x) - 1.0;
  float y_scaled = 2.0 * (float)(v.y - m_min.y) / (m_max.y - m_min.y) - 1.0;

  float angle = atan2(y_scaled, x_scaled) * 180 / M_PI;
  if (angle < 0)
    angle += 360;
  return angle;
}

// Yields the angle difference in degrees between two headings
float relativeHeading(float heading_from, float heading_to) {
  float relative_heading = heading_to - heading_from;

  // constrain to -180 to 180 degree range
  if (relative_heading > 180)
    relative_heading -= 360;
  if (relative_heading < -180)
    relative_heading += 360;

  return relative_heading;
}

// Average 10 vectors to get a better measurement and help smooth out
// the motors' magnetic interference.
float averageHeading() {
  IMU::vector<int32_t> avg = { 0, 0, 0 };

  for (int i = 0; i < 10; i++) {
    imu.readMag();
    avg.x += imu.m.x;
    avg.y += imu.m.y;
  }
  avg.x /= 10.0;
  avg.y /= 10.0;

  // avg is the average measure of the magnetic vector.
  return heading(avg);
}

// void selectHyper() {
//   motors.flipLeftMotor(true);
//   motors.flipRightMotor(true);
//   // Encoders are not used in this example.
//   // encoders.flipEncoders(true);

//   // The right motor tends to be a bit slower on the Hyper edition.
//   // Speed it up to drive straighter.
//   // You might need to adjust this value for your particular unit.
//   speedStraightLeft = 70;
//   speedStraightRight = 80;
//   turnBaseSpeed = 20;
// }

void selectStandard() {
  speedStraightLeft = 100;
  speedStraightRight = speedStraightLeft;
  turnBaseSpeed = 30;
}

// void selectTurtle() {
//   speedStraightLeft = 200;
//   speedStraightRight = speedStraightLeft;
//   turnBaseSpeed = 40;
// }

void calibrateTurnSensor() {
  IMU::vector<int16_t> running_min = { 32767, 32767, 32767 }, running_max = { -32767, -32767, -32767 };
  unsigned char index;

  // Initialize the Wire library and join the I2C bus as a master
  Wire.begin();
  // Initialize IMU
  imu.init();
  // Enables accelerometer and magnetometer
  imu.enableDefault();
  imu.configureForCompassHeading();

  selectStandard();
  delay(1000);
  Serial.println("Setup Complete");

  // To calibrate the magnetometer, the 3pi+ spins to find the max/min
  // magnetic vectors. This information is used to correct for offsets
  // in the magnetometer data.
  motors.setLeftSpeed(speedStraightLeft);
  motors.setRightSpeed(-speedStraightRight);

  for (index = 0; index < CALIBRATION_SAMPLES; index++) {
    // Take a reading of the magnetic vector and store it in compass.m
    imu.readMag();

    running_min.x = min(running_min.x, imu.m.x);
    running_min.y = min(running_min.y, imu.m.y);

    running_max.x = max(running_max.x, imu.m.x);
    running_max.y = max(running_max.y, imu.m.y);

    Serial.println(index);

    delay(50);
  }

  // Store calibrated values in m_max and m_min
  m_max.x = running_max.x;
  m_max.y = running_max.y;
  m_min.x = running_min.x;
  m_min.y = running_min.y;
}



void turnTwo(char dir) {
  float heading, relative_heading;
  int speed;
  static float target_heading = 0;

  // Heading is given in degrees away from the magnetic vector, increasing clockwise
  heading = averageHeading();

  // This gives us the relative heading with respect to the target angle
  relative_heading = relativeHeading(heading, target_heading);

  if (dir == 'l') {
    target_heading = heading - 90;
  } else if (dir == 'r') {
    target_heading = heading + 90;
  }

  if (target_heading >= 360) {
    target_heading -= 360;
  }

  if (target_heading < 0) {
    target_heading += 360;
  }

  // If the 3pi+ has turned to the direction it wants to be pointing, go straight and then do another turn
  // while (heading >= (target_heading + DEVIATION_THRESHOLD) || heading <= (target_heading - DEVIATION_THRESHOLD)) {
  while (relative_heading > DEVIATION_THRESHOLD || relative_heading < -DEVIATION_THRESHOLD) {
    speed = speedStraightLeft * relative_heading / 180;

    if (speed < 0)
      speed -= turnBaseSpeed;
    else
      speed += turnBaseSpeed;

    // motors.setSpeeds(speed, -speed);
    heading = averageHeading();
    relative_heading = relativeHeading(heading, target_heading);

    display.clear();
    display.gotoXY(0, 0);
    display.print(heading);
    display.print(F("   "));
  }
  motors.setSpeeds(0, 0);
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
