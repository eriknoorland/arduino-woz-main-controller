#include "Arduino.h"
#include "Ramp.h"
#include "MotorController.h"

ramp speedRamp;

float Kp = 0.065;
float Ki = 0.003;
float Kd = 0.01;

/**
 * MotorController
 * @param maxSpeed
 * @param numTicksPerRevolution
 * @param wheelCircumference
 * @param loopTime
 */
MotorController::MotorController(int maxSpeed, float numTicksPerRevolution, float wheelCircumference, int loopTime) {
  _maxSpeed = maxSpeed;
  _numTicksPerRevolution = numTicksPerRevolution;
  _wheelCircumference = wheelCircumference;
  _loopTime = loopTime;

  _numTicks = 0;
  _numLastTicks = _numTicks;
  _goalSpeed = 0;
  _direction = 0;
  _lastError = 0;
  _iAcc = 0;
  _hardStop = false;
}

/**
 * Setup
 * @param enable
 * @param enableB
 * @param pwm1
 * @param pwm2
 * @param encoderA
 * @param encoderB
 */
void MotorController::setup(int enable, int enableB, int pwm1, int pwm2, int encoderA, int encoderB) {
  _pwm1Pin = pwm1;
  _pwm2Pin = pwm2;

  pinMode(enable, OUTPUT);
  pinMode(enableB, OUTPUT);
  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);

  digitalWrite(enable, HIGH);
  digitalWrite(enableB, LOW);
}

/**
 * Loop
 */
void MotorController::loop() {
  speedRamp.update();
}

/**
 * Move
 * @param speed
 * @param direction
 * @param accelerationDuration
 */
void MotorController::move(int speed, int direction, int accelerationDuration) {
  float revolutionsPerSecond = speed / _wheelCircumference;
  float ticksPerSecond = _numTicksPerRevolution * revolutionsPerSecond;

  _goalSpeed = ticksPerSecond / (1000 / _loopTime);
  _direction = direction;
  _hardStop = false;

  speedRamp.go(_goalSpeed, accelerationDuration, LINEAR);
}

/**
 * Stop
 * @param hard
 * @param decelerationDuration
 */
void MotorController::stop(bool hard, int decelerationDuration) {
  int duration = hard ? 1 : decelerationDuration;

  _goalSpeed = 0;
  _hardStop = hard;

  speedRamp.go(_goalSpeed, duration, LINEAR);
}

/**
 * Change speed
 * @param speed
 * @param decelerationDuration
 */
void MotorController::changeSpeed(int speed, int decelerationDuration) {
  _goalSpeed = speed;

  speedRamp.go(_goalSpeed, decelerationDuration, LINEAR);
}

/**
 * Motor control
 * @param {int} pwm1
 * @param {int} pwm2
 */
void MotorController::motorControl(int pwm1, int pwm2) {
  analogWrite(_pwm1Pin, _direction == 1 ? pwm1 : 0);
  analogWrite(_pwm2Pin, _direction == -1 ? pwm2 : 0);
}

/**
 * Timer interrupt event handler
 * @return int
 */
int MotorController::onTimerInterrupt() {
  int deltaTicks = _numTicks - _numLastTicks;
  int error = _goalSpeed - deltaTicks;
  float p = Kp * error;
  float i = _iAcc + (_loopTime * error * Ki);
  float d = Kd * ((error - _lastError) / _loopTime);
  int ticksSpeed = p + i + d;

  _numLastTicks = _numTicks;
  _lastError = error;
  _iAcc = i;

  int pwmSpeed = constrain(ticksToPwm(ticksSpeed, _maxSpeed), 0, 255);

  if (_hardStop) {
    deltaTicks = 0;
    pwmSpeed = 0;
  }

  motorControl(
    _direction == 1 ? constrain(pwmSpeed, 0, 255) : 0,
    _direction == -1 ? constrain(pwmSpeed, 0, 255) : 0
  );

  return deltaTicks;
}

/**
 * Encoder tick event handler
 */
void MotorController::onEncoderTick() {
  _numTicks += 1;
}

/**
 * Returns a pwm value based on the given tick and max speed
 * @param {int} tickSpeed
 * @param {int} maxSpeed
 * @return
 */
int MotorController::ticksToPwm(int tickSpeed, int maxSpeed) {
  return round(tickSpeed * 255 / maxSpeed);
}
