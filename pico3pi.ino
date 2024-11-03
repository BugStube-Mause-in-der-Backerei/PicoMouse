#include <Pololu3piPlus2040.h>

#define CALIBRATION_SAMPLES 70  // Number of compass readings to take when calibrating

// Allowed deviation (in degrees) relative to target angle that must be achieved before driving straight
#define DEVIATION_THRESHOLD 1

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

unsigned long currentMillis;
unsigned long prevMillis;
const unsigned long PERIOD = 20;
long countsLeft = 0;
long countsRight = 0;
long prevLeft = 0;
long prevRight = 0;
const int CLICKS_PER_ROTATION = 12;
const float GEAR_RATIO = 29.86F;
const int WHEEL_CIRCUMFERENZCE = 10.0531;
float Sl = 0.0F;
float Sr = 0.0F;
int wheelSpeed = 75;

uint16_t speedStraightLeft;  // Maximum motor speed when going straight; variable speed when turning
uint16_t speedStraightRight;
uint16_t turnBaseSpeed;  // Base speed when turning (added to variable speed)
uint16_t driveTime;      // Time to drive straight, in milliseconds

IMU::vector<int16_t> m_max;  // maximum magnetometer values, used for calibration
IMU::vector<int16_t> m_min;  // minimum magnetometer values, used for calibration



String inputs[] = { "forward" };
int inputSize = sizeof(inputs) / sizeof(String);

String inputsC[] = { "forward", "turnright", "forward", "turnright", "forward", "turnright", "forward", "turnright" };
int inputSizeC = sizeof(inputsC) / sizeof(String);

void setup() {
  // motors.flipLeftMotor(true);
  // motors.flipRightMotor(true);

  Serial.begin(9600);
  delay(1000);
  buzzer.play("C32");

  bumpSensors.calibrate();
  // calibrateTurnSensor();
  
}

void loop() {
  // Serial.println("Working...");
  motors.setSpeeds(0, 0);
  bumpSensors.read();
  // put your main code here, to run repeatedly:
  if (buttonA.isPressed()) {
    Serial.println("PRESSED A");
    delay(2000);
    for (int i = 0; i < inputSize; i++) {
      handleInput(inputs[i]);
    }
  }


  if (buttonC.isPressed()) {
    delay(2000);
    for (int i = 0; i < inputSizeC; i++) {
      handleInput(inputsC[i]);
    }
  }
}

void handleInput(String input) {
  if (input.equalsIgnoreCase("forward")) {
    Serial.println("moveForward");
    moveForward();
  } else if (input.equalsIgnoreCase("turnleft")) {
    Serial.println("turnleft");
    turn('l');
  } else if (input.equalsIgnoreCase("turnright")) {
    Serial.println("turnright");
    turn('r');
  }
  delay(50);
}

void turn(char dir) {
  int turnSpeed = 80;
  int turnSpeedNeg = -80;

  if (dir == 'l') {
    motors.setSpeeds(turnSpeedNeg, turnSpeed);
  } else if (dir == 'r') {
    motors.setSpeeds(turnSpeed, turnSpeedNeg);
  }
  delay(255);
  motors.setSpeeds(0, 0);
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
  while (target_heading > (heading + DEVIATION_THRESHOLD) || target_heading < (heading - DEVIATION_THRESHOLD)) {
    speed = speedStraightLeft*relative_heading/180;

    if (speed < 0)
      speed -= turnBaseSpeed;
    else
      speed += turnBaseSpeed;

    motors.setSpeeds(speed, -speed);
    heading = averageHeading();
    relative_heading = relativeHeading(heading, target_heading);
  }
}

void moveForward() {
  Sr = 0.0F;

  while (!bumpSensors.leftIsPressed() && !bumpSensors.rightIsPressed()) {
    countsRight += encoders.getCountsAndResetRight();

    Sr += ((countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENZCE);

    if (Sr < 16) {
      motors.setSpeeds(wheelSpeed, wheelSpeed);
    } else {
      break;
    }

    prevRight = countsRight;
    bumpSensors.read();
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
//   driveTime = 500;
// }

void selectStandard() {
  speedStraightLeft = 100;
  speedStraightRight = speedStraightLeft;
  turnBaseSpeed = 40;
  driveTime = 1000;
}

// void selectTurtle() {
//   speedStraightLeft = 200;
//   speedStraightRight = speedStraightLeft;
//   turnBaseSpeed = 40;
//   driveTime = 2000;
// }

void calibrateTurnSensor(){
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
