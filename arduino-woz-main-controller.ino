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
#define REQUEST_KEEP_HEADING 0x16
#define REQUEST_RESET_IMU 0x20
#define REQUEST_IS_READY 0x30
#define REQUEST_SET_DATA 0x31

// response bytes
#define RESPONSE_START_FLAG_1 0xA3
#define RESPONSE_START_FLAG_2 0x3A
#define RESPONSE_ODOMETRY 0x30
#define RESPONSE_READY 0xFF

int frequency = 50; // Hz
int loopTime = 1000 / frequency; // ms
int maxRPM = 160;
int encoderCPR = 48; // two pin encoder, double edge
float gearRatio = 46.85;
float wheelBase = 168.08; // mm
float wheelDiameter = 60; // mm
float wheelBaseCircumference = PI * wheelBase; // mm
float wheelCircumference = PI * wheelDiameter; // mm
float numTicksPerRevolution = gearRatio * encoderCPR;
float distancePerTick = wheelCircumference / numTicksPerRevolution; // mm
int maxTickSpeed = (int) ((maxRPM * numTicksPerRevolution) / 60) / (1000 / loopTime); // ticks/loopTime
int minSpeed = 10; // mm/s
int maxSpeed = 430; // mm/s

int leftMotorSpeed = 0; // mm/s
int rightMotorSpeed = 0; // mm/s
int leftMotorDirection = 0;
int rightMotorDirection = 0;

bool isImuDetected = false;
float lastPhi = 0.0;

MotorController leftMotorController = MotorController(maxTickSpeed, numTicksPerRevolution, wheelCircumference, loopTime);
MotorController rightMotorController = MotorController(maxTickSpeed, numTicksPerRevolution, wheelCircumference, loopTime);

Adafruit_BNO055 bno = Adafruit_BNO055(55);

PacketSerial serial;

struct MotionTarget {
  int command;
  int heading;
  bool isDecelerating;
  int decelerationDuration;
  int numDecelerationTargetTicks;
  int numTargetTicks;
  int accLeftTicks;
  int accRightTicks;
};

MotionTarget motionTarget;

/**
 * Resets the motion target to defaul values
 */
void resetMotionTarget() {
  motionTarget = { 0, -1, false, 0, 0, 0, 0, 0 };
}

/**
 * Returns the number of ticks required for the given distance
 * @param {int} distance
 * @return int
 */
int getTargetTicks(int distance) {
  return round(distance / distancePerTick);
}

/**
 * Returns the deceleration duration based on the current and desired speed
 * @param {int} currentSpeed
 * @param {int} speed
 * @param {int} maxDecelerationDuration (ms)
 * @return int
 */
int getDecelerationDuration(int currentSpeed, int speed, int maxDecelerationDuration = 4000) {
  return round(map(abs(currentSpeed - speed), 0, maxSpeed, 0, maxDecelerationDuration));
}

/**
 * Returns the deceleration distance for the given speed and deceleration duration
 * @param {int} currentSpeed
 * @param {int} speed
 * @param {int} decelerationDuration
 * @return
 */
float getDecelerationDistance(int currentSpeed, int speed, int decelerationDuration) {
  int speedDiff = abs(speed - currentSpeed);
  float decelerationDistance = (speedDiff * ((float) decelerationDuration / 1000)) / 2;

  return decelerationDistance;
}

/**
 * Returns a signed difference between the given angles
 * @param {int} target
 * @param {int} current
 * @return
 */
int getRelativeAngleDifference(int target, int current) {
  return ((target - current + (360 + 180)) % 360) - 180;
};

/**
 * Straight
 * @param {int} speed
 * @param {int} direction
 * @param {int} distance
 */
void straight(int speed, int direction, int distance = 0) {
  speed = constrain(speed, minSpeed, maxSpeed);

  leftMotorDirection = direction == 1 ? 1 : -1;
  rightMotorDirection = direction == 1 ? -1 : 1;

  int decelerationDuration = getDecelerationDuration(leftMotorSpeed, speed); // FIXME current speed parameter

  motionTarget.command = direction == 1 ? REQUEST_FORWARD : REQUEST_REVERSE;
  motionTarget.accLeftTicks = 0;
  motionTarget.accRightTicks = 0;
  motionTarget.isDecelerating = false;
  motionTarget.decelerationDuration = decelerationDuration;

  if (distance != 0) {
    int decelerationOffset = 10; // round(speed / 8); // mm
    float decelerationDistance = getDecelerationDistance(0, speed - minSpeed, decelerationDuration); // mm

    motionTarget.numDecelerationTargetTicks = getTargetTicks(distance - decelerationDistance - decelerationOffset) - 250;
    motionTarget.numTargetTicks = getTargetTicks(distance);
  }

  leftMotorController.move(speed, leftMotorDirection, decelerationDuration);
  rightMotorController.move(speed, rightMotorDirection, decelerationDuration);
}

/**
 * Keep heading
 * @param {int} speed
 * @param {int} heading
 * @param {int} direction
 * @param {int} distance
 */
void keepHeading(int speed, int heading, int direction, int distance = 0) {
  motionTarget.heading = heading;

  straight(speed, direction == 1 ? 1 : -1, distance);

  motionTarget.command = REQUEST_KEEP_HEADING;
}

/**
 * Rotate
 * @param {int} speed
 * @param {int} angle
 * @param {int} direction
 */
void rotate(int speed, int angle, int direction = 0) {
  leftMotorDirection = direction == 1 ? -1 : 1;
  rightMotorDirection = direction == 1 ? -1 : 1;

  int decelerationOffset = 10; // mm
  int decelerationDuration = 1000; // getDecelerationDuration(speed, 500);
  float distance = wheelBaseCircumference / (360 / (float) angle); // mm
  float decelerationDistance = getDecelerationDistance(0, speed - minSpeed, decelerationDuration);

  motionTarget.command = REQUEST_ROTATE;
  motionTarget.accLeftTicks = 0;
  motionTarget.accRightTicks = 0;
  motionTarget.isDecelerating = false;
  motionTarget.decelerationDuration = decelerationDuration;

  motionTarget.numDecelerationTargetTicks = getTargetTicks(distance - decelerationDistance - decelerationOffset) - 65;
  motionTarget.numTargetTicks = getTargetTicks(distance);

  leftMotorController.move(speed, leftMotorDirection, decelerationDuration);
  rightMotorController.move(speed, rightMotorDirection, decelerationDuration);
}

/**
 * Turn
 * @param {int} speed
 * @param {int} angle
 * @param {int} radius
 * @param {int} direction
 */
void turn(int speed, int angle, int radius, int direction = 0) {
  leftMotorDirection = direction == 1 ? -1 : 1;
  rightMotorDirection = direction == 1 ? 1 : -1;

  motionTarget.command = REQUEST_TURN;

  int speedLeft = speed; // FIXME calculate left speed based on radius
  int speedRight = speed; // FIXME calculate right speed based on radius

  // leftMotorController.move(speedLeft, leftMotorDirection);
  // rightMotorController.move(speedRight, rightMotorDirection);
}

/**
 * Drive
 * @param {int} speedLeft
 * @param {int} speedRight
 */
void drive(int speedLeft, int speedRight) {
  leftMotorDirection = 1;
  rightMotorDirection = -1;

  // leftMotorController.move(speedLeft, leftMotorDirection);
  // rightMotorController.move(speedRight, rightMotorDirection);
}

/**
 * Stop
 * @param {bool} hard
 */
void stop(bool hard = false) {
  int decelerationDuration = round(map(leftMotorSpeed, 0, maxSpeed, 0, 2000)); // FIXME current speed parameter
  // int decelerationDuration = getDecelerationDuration(leftMotorSpeed, 0); // FIXME current speed parameter

  leftMotorController.stop(hard, hard ? 1 : decelerationDuration);
  rightMotorController.stop(hard, hard ? 1 : decelerationDuration);

  if (hard) {
    leftMotorSpeed = 0;
    rightMotorSpeed = 0;
    resetMotionTarget();
  }

  motionTarget.heading = -1;
}

/**
 * Resets the IMU
 */
void resetIMU() {
  isImuDetected = false;

  digitalWrite(IMU_RESET_PIN, LOW);
  delayMicroseconds(1);
  digitalWrite(IMU_RESET_PIN, HIGH);

  if (bno.begin()) {
    isImuDetected = true;
  }
}

/**
 * Send the ready response
 */
void isReady() {
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
 * Heading handler
 * @param {int} current
 * @param {int} target
 * @param {int} numLeftTicks
 * @param {int} numRightTicks
 */
void headingHandler(int current, int target, int numLeftTicks, int numRightTicks) {
  float distanceLeft = distancePerTick * numLeftTicks;
  float distanceRight = distancePerTick * numRightTicks;
  float phi = lastPhi - ((distanceRight - distanceLeft) / wheelBase);

  // Serial.print(distanceLeft);
  // Serial.print(" \t ");
  // Serial.print(distanceRight);
  // Serial.print(" \t ");
  // Serial.print(distanceRight - distanceLeft);
  // Serial.print(" \t ");
  // Serial.print(distancePerTick);
  // Serial.print(" \t ");
  // Serial.println(phi);

  lastPhi = phi;

  // get pose
  // use pose phi to verify heading
  // use pid to correct speed

  // int headingDifference = getRelativeAngleDifference(current, target);
  // int leftSpeedCorrection = headingDifference > 0 ? headingDifference * -1 : 0;
  // int rightSpeedCorrection = headingDifference < 0 ? headingDifference * -1 : 0;
  // int leftMotorSpeed = leftMotorController.getSpeed() + leftSpeedCorrection;
  // int rightMotorSpeed = rightMotorController.getSpeed() + rightSpeedCorrection;

  // leftMotorSpeed = constrain(leftMotorSpeed, minSpeed, maxSpeed);
  // rightMotorSpeed = constrain(rightMotorSpeed, minSpeed, maxSpeed);

  // leftMotorController.correctSpeed(leftMotorSpeed);
  // rightMotorController.correctSpeed(rightMotorSpeed);
}

/**
 * Distance handler
 * @param {int} numLeftTicks
 * @param {int} numRightTicks
 */
void distanceHandler(int numLeftTicks, int numRightTicks) {
  motionTarget.accLeftTicks += numLeftTicks;
  motionTarget.accRightTicks += numRightTicks;

  int accTicks = (motionTarget.accLeftTicks + motionTarget.accRightTicks) / 2;

  if (!motionTarget.isDecelerating && accTicks >= motionTarget.numDecelerationTargetTicks) {
    motionTarget.isDecelerating = true;

    leftMotorController.changeSpeed(minSpeed, motionTarget.decelerationDuration);
    rightMotorController.changeSpeed(minSpeed, motionTarget.decelerationDuration);
  }

  if (accTicks >= motionTarget.numTargetTicks) {
    if (motionTarget.command != 0) {
      uint8_t response[4] = {
        RESPONSE_START_FLAG_1,
        RESPONSE_START_FLAG_2,
        motionTarget.command,
        0x00
      };

      serial.send(response, sizeof(response));
    }

    stop(true);
    resetMotionTarget();
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
      case REQUEST_KEEP_HEADING: {
        int speed = (buffer[2] << 8) + buffer[3];
        int distance = (buffer[6] << 8) + buffer[7];

        keepHeading(speed, buffer[4], bitRead(buffer[5], 0), distance);
        break;
      }

      case REQUEST_FORWARD: {
        int speed = (buffer[2] << 8) + buffer[3];
        int distance = (buffer[4] << 8) + buffer[5];

        straight(speed, 1, distance);
        break;
      }

      case REQUEST_REVERSE: {
        int speed = (buffer[2] << 8) + buffer[3];
        int distance = (buffer[4] << 8) + buffer[5];

        straight(speed, -1, distance);
        break;
      }

      case REQUEST_ROTATE: {
        int speed = (buffer[2] << 8) + buffer[3];

        rotate(speed, buffer[4], bitRead(buffer[5], 0));
        break;
      }

      case REQUEST_TURN: {
        int speed = (buffer[2] << 8) + buffer[3];
        // FIXME
        break;
      }

      case REQUEST_DRIVE: {
        int speedLeft = (buffer[2] << 8) + buffer[3];
        int speedRight = (buffer[4] << 8) + buffer[5];

        drive(speedLeft, speedRight);
        break;
      }

      case REQUEST_STOP: {
        stop(buffer[2]);
        break;
      }

      case REQUEST_RESET_IMU: {
        resetIMU();
        break;
      }

      case REQUEST_IS_READY: {
        isReady();
        break;
      }

      case REQUEST_SET_DATA: {
        break;
      }
    }
  }
}

/**
 * Timer interrupt handler
 */
void onTimerInterrupt() {
  int deltaLeftTicks = leftMotorController.onTimerInterrupt();
  int deltaRightTicks = rightMotorController.onTimerInterrupt();

  leftMotorSpeed = deltaLeftTicks * distancePerTick * frequency; // mm/s
  rightMotorSpeed = deltaLeftTicks * distancePerTick * frequency; // mm/s

  if (motionTarget.numTargetTicks != 0) {
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

    // if (motionTarget.heading != -1) {
    //   headingHandler(round(heading / 100), motionTarget.heading, deltaLeftTicks, deltaRightTicks);
    // }
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

  analogWriteResolution(10);

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

  resetMotionTarget();
  isReady();
}

/**
 * Loop
 */
void loop() {
  leftMotorController.loop();
  rightMotorController.loop();
  serial.update();
}
