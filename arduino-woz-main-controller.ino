#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "PacketSerial.h"
#include "TimerOne.h"
#include "MotorController.h"

// pins
#define IMU_RESET_PIN 20
#define MOTOR_LEFT_ENCODER_A_PIN 17
#define MOTOR_LEFT_ENCODER_B_PIN 16
#define MOTOR_RIGHT_ENCODER_A_PIN 15
#define MOTOR_RIGHT_ENCODER_B_PIN 14

// request bytes
#define REQUEST_START_FLAG 0xA3
#define REQUEST_FORWARD 0x10
#define REQUEST_REVERSE 0x11
#define REQUEST_ROTATE 0x12
#define REQUEST_TURN 0x13
#define REQUEST_DRIVE 0x14
#define REQUEST_STOP 0x15
#define REQUEST_RESET_IMU 0x20

// response bytes
#define RESPONSE_START_FLAG_1 0xA3
#define RESPONSE_START_FLAG_2 0x3A
#define RESPONSE_ODOMETRY 0x30
#define RESPONSE_READY 0xFF

int loopTime = 1000 / 50; // ms
int maxRPM = 160;
int encoderCPR = 48; // (two pin encoder, double edge)
float gearRatio = 46.85;
float wheelBase = 16.75; // cm
float wheelBaseCircumference = PI * wheelBase; // cm
float wheelCircumference = PI * 6; // cm
float numTicksPerRevolution = gearRatio * encoderCPR;
float distancePerTick = wheelCircumference / numTicksPerRevolution; // cm
int maxTickSpeed = (int) ((maxRPM * numTicksPerRevolution) / 60) / (1000 / loopTime); // ticks/loopTime
int minSpeed = 3; // cm/s
int maxSpeed = 43; // cm/s
int currentSpeed = 0; // cm/s
int maxDecelerationDuration = 2000; // ms
int decelerationOffset = 0; // cm

bool isImuDetected = false;
int leftMotorDirection = 0;
int rightMotorDirection = 0;
int targetCommand = 0;
int accTicks = 0;

bool isDecelerating = false;
int numDecelerationTargetTicks = 0;
int numTargetTicks = 0;

MotorController leftMotorController(maxTickSpeed, numTicksPerRevolution, wheelCircumference, loopTime);
MotorController rightMotorController(maxTickSpeed, numTicksPerRevolution, wheelCircumference, loopTime);

Adafruit_BNO055 bno = Adafruit_BNO055(55);

PacketSerial serial;

/**
 * Returns the number of ticks required for the given distance
 * @param distance
 * @return int
 */
int getTargetTicks(int distance) {
  return round(distance / distancePerTick);
}

/**
 * Returns the deceleration duration based on the current and desired speed
 * @param speed
 * @return int
 */
int getDecelerationDuration(int speed) {
  return round(map(abs(currentSpeed - speed), 0, maxSpeed, 0, maxDecelerationDuration));
}

/**
 * Returns the deceleration distance for the given speed and deceleration duration
 * @param speed
 * @param decelerationDuration
 * @return
 */
float getDecelerationDistance(int speed, int decelerationDuration) {
  int speedDiff = abs(speed - currentSpeed);
  float decelerationDistance = (speedDiff * ((float) decelerationDuration / 1000)) / 2;

  return decelerationDistance;
}

/**
 * Forward
 * @param speed
 * @param distance
 */
void forward(int speed, int distance = 0) {
  speed = constrain(speed, minSpeed, maxSpeed);

  leftMotorDirection = 1;
  rightMotorDirection = -1;

  targetCommand = REQUEST_FORWARD;
  isDecelerating = false;
  accTicks = 0;

  int accelerationDuration = getDecelerationDuration(speed);

  if (distance != 0) {
    float decelerationDistance = getDecelerationDistance(speed - minSpeed, accelerationDuration);
    numDecelerationTargetTicks = getTargetTicks(distance - decelerationDistance - decelerationOffset);
    numTargetTicks = getTargetTicks(distance);
  }

  leftMotorController.move(speed, leftMotorDirection, accelerationDuration);
  rightMotorController.move(speed, rightMotorDirection, accelerationDuration);
}

/**
 * Reverse
 * @param speed
 * @param distance
 */
void reverse(int speed, int distance = 0) {
  speed = constrain(speed, minSpeed, maxSpeed);

  leftMotorDirection = -1;
  rightMotorDirection = 1;

  targetCommand = REQUEST_REVERSE;
  isDecelerating = false;
  accTicks = 0;

  int accelerationDuration = getDecelerationDuration(speed);

  if (distance != 0) {
    float decelerationDistance = getDecelerationDistance(speed - minSpeed, accelerationDuration);
    numDecelerationTargetTicks = getTargetTicks(distance - decelerationDistance - decelerationOffset);
    numTargetTicks = getTargetTicks(distance);
  }

  leftMotorController.move(speed, leftMotorDirection, accelerationDuration);
  rightMotorController.move(speed, rightMotorDirection, accelerationDuration);
}

/**
 * Rotate
 * @param speed
 * @param angle
 * @param direction
 */
void rotate(int speed, int angle, int direction = 0) {
  leftMotorDirection = direction == 1 ? -1 : 1;
  rightMotorDirection = direction == 1 ? -1 : 1;

  targetCommand = REQUEST_ROTATE;
  isDecelerating = false;
  accTicks = 0;

  float distance = wheelBaseCircumference / (360 / (float) angle);
  int accelerationDuration = getDecelerationDuration(speed);
  float decelerationDistance = getDecelerationDistance(speed - minSpeed, accelerationDuration);
  numDecelerationTargetTicks = getTargetTicks(distance - decelerationDistance - decelerationOffset);
  numTargetTicks = getTargetTicks(distance);

  leftMotorController.move(speed, leftMotorDirection, accelerationDuration);
  rightMotorController.move(speed, rightMotorDirection, accelerationDuration);
}

/**
 * Turn
 * @param speed
 * @param angle
 * @param radius
 * @param direction
 */
void turn(int speed, int angle, int radius, int direction = 0) {
  leftMotorDirection = direction == 1 ? -1 : 1;
  rightMotorDirection = direction == 1 ? 1 : -1;

  targetCommand = REQUEST_TURN;

  int speedLeft = speed; // FIXME calculate left speed based on radius
  int speedRight = speed; // FIXME calculate right speed based on radius

  // leftMotorController.move(speedLeft, leftMotorDirection);
  // rightMotorController.move(speedRight, rightMotorDirection);
}

/**
 * Drive
 * @param speedLeft
 * @param speedRight
 */
void drive(int speedLeft, int speedRight) {
  leftMotorDirection = 1;
  rightMotorDirection = -1;

  // leftMotorController.move(speedLeft, leftMotorDirection);
  // rightMotorController.move(speedRight, rightMotorDirection);
}

/**
 * Stop
 * @param hard
 */
void stop(bool hard = false) {
  int decelerationDuration = getDecelerationDuration(0);

  leftMotorController.stop(hard, hard ? 1 : decelerationDuration);
  rightMotorController.stop(hard, hard ? 1 : decelerationDuration);

  if (hard) {
    currentSpeed = 0;
    isDecelerating = false;
  }
}

/**
 * Resets the IMU
 */
void resetIMU() {
  digitalWrite(IMU_RESET_PIN, LOW);
  delay(1);
  digitalWrite(IMU_RESET_PIN, HIGH);
}

/**
 * Distance handler
 */
void distanceHandler(int leftTicks, int rightTicks) {
  accTicks += (leftTicks + rightTicks) / 2;

  if (!isDecelerating && accTicks >= numDecelerationTargetTicks) {
    isDecelerating = true;

    int decelerationDuration = getDecelerationDuration(minSpeed);

    leftMotorController.changeSpeed(minSpeed, decelerationDuration);
    rightMotorController.changeSpeed(minSpeed, decelerationDuration);
  }

  if (accTicks >= numTargetTicks) {
    stop(true);

    if (targetCommand != 0) {
      uint8_t response[4] = {
        RESPONSE_START_FLAG_1,
        RESPONSE_START_FLAG_2,
        targetCommand,
        0x00
      };

      serial.send(response, sizeof(response));
    }

    targetCommand = 0;
    numTargetTicks = 0;
  }
}

/**
 * Serial packet received event handler
 * @param {uint8_t} buffer
 * @param {size_t} size
 */
void onPacketReceived(const uint8_t* buffer, size_t size) {
  byte startFlag = buffer[0];
  byte command = buffer[1];

  if (startFlag == REQUEST_START_FLAG) {
    switch (command) {
      case REQUEST_FORWARD:
        forward(buffer[2], buffer[3]);
        break;
      case REQUEST_REVERSE:
        reverse(buffer[2], buffer[3]);
        break;
      case REQUEST_ROTATE:
        rotate(buffer[2], buffer[3], bitRead(buffer[4], 0));
        break;
      case REQUEST_TURN:
        // FIXME
        break;
      case REQUEST_DRIVE:
        drive(buffer[2], buffer[3]);
        break;
      case REQUEST_STOP:
        stop(buffer[2]);
        break;
      case REQUEST_RESET_IMU:
        resetIMU();
        break;
    }
  }
}

/**
 * Timer interrupt handler
 */
void onTimerInterrupt() {
  byte deltaLeftTicks = leftMotorController.onTimerInterrupt();
  byte deltaRightTicks = rightMotorController.onTimerInterrupt();

  currentSpeed = round((((deltaLeftTicks + deltaRightTicks) / 2) * distancePerTick) * 50); // cm/s

  if (numTargetTicks != 0) {
    distanceHandler(deltaLeftTicks, deltaRightTicks);
  }

  byte headingPartMsb = 0;
  byte headingPartLsb = 0;

  if (isImuDetected) {
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

    int heading = orientationData.orientation.x * 100;
    headingPartMsb = heading >> 8;
    headingPartLsb = heading;
  }

  uint8_t response[10] = {
    RESPONSE_START_FLAG_1,
    RESPONSE_START_FLAG_2,
    RESPONSE_ODOMETRY,
    0x06,
    leftMotorDirection < 0 ? 0 : 1,
    deltaLeftTicks,
    rightMotorDirection < 0 ? 1 : 0,
    deltaRightTicks,
    headingPartMsb,
    headingPartLsb
  };

  serial.send(response, sizeof(response));
}

/**
 * Left encoder tick event handler
 */
void onLeftEncoderTick() {
  leftMotorController.onEncoderTick();
}

/**
 * Right encoder tick event handler
 */
void onRightEncoderTick() {
  rightMotorController.onEncoderTick();
}

/**
 * Setup
 */
void setup() {
  Serial.begin(115200);
  serial.setStream(&Serial);
  serial.setPacketHandler(&onPacketReceived);

  leftMotorController.setup(4, 5, 6, 12, MOTOR_LEFT_ENCODER_A_PIN, MOTOR_LEFT_ENCODER_B_PIN);
  rightMotorController.setup(8, 9, 10, 11, MOTOR_RIGHT_ENCODER_A_PIN, MOTOR_RIGHT_ENCODER_B_PIN);

  pinMode(IMU_RESET_PIN, OUTPUT);
  digitalWrite(IMU_RESET_PIN, HIGH);

  if (bno.begin()) {
    isImuDetected = true;
  }

  Timer1.initialize(loopTime * 1000);
  Timer1.attachInterrupt(onTimerInterrupt);

  attachInterrupt(MOTOR_LEFT_ENCODER_A_PIN, onLeftEncoderTick, CHANGE);
  attachInterrupt(MOTOR_LEFT_ENCODER_B_PIN, onLeftEncoderTick, CHANGE);
  attachInterrupt(MOTOR_RIGHT_ENCODER_A_PIN, onRightEncoderTick, CHANGE);
  attachInterrupt(MOTOR_RIGHT_ENCODER_B_PIN, onRightEncoderTick, CHANGE);

  while (!Serial) {}

  uint8_t readyResponse[5] = {
    RESPONSE_START_FLAG_1,
    RESPONSE_START_FLAG_2,
    RESPONSE_READY,
    0x01,
    isImuDetected
  };

  serial.send(readyResponse, sizeof(readyResponse));
}

/**
 * Loop
 */
void loop() {
  leftMotorController.loop();
  rightMotorController.loop();
  serial.update();
}
