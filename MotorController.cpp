#include "Arduino.h"
#include "MotorController.h"

float Kp = 0.065;
float Ki = 0.04;
float Kd = 0.0025;

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
  _numLastTicks = 0;
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
  _speedRamp.update();
}

/**
 * Correct speed
 * @param speed
 */
void MotorController::correctSpeed(int speed) {
  float revolutionsPerSecond = speed / _wheelCircumference;
  float ticksPerSecond = _numTicksPerRevolution * revolutionsPerSecond;
  int ticksSpeed = ticksPerSecond / (1000 / _loopTime);
  int pwmSpeed = constrain(ticksToPwm(ticksSpeed, _maxSpeed), 0, 255);

  motorControl(
    _direction == 1 ? constrain(pwmSpeed, 0, 255) : 0,
    _direction == -1 ? constrain(pwmSpeed, 0, 255) : 0
  );
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

  _speedRamp.go(_goalSpeed, accelerationDuration, LINEAR);
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

  _speedRamp.go(_goalSpeed, duration, LINEAR);
}

/**
 * Change speed
 * @param speed
 * @param decelerationDuration
 */
void MotorController::changeSpeed(int speed, int decelerationDuration) {
  move(speed, _direction, decelerationDuration);
}

/**
 * Returns the current speed
 * @return int
 */
int MotorController::getSpeed() {
  int tickSpeed = _speedRamp.value();
  int ticksPerSecond = tickSpeed * (1000 / _loopTime);
  int speed = round(ticksPerSecond / (_numTicksPerRevolution / _wheelCircumference));

  return speed;
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
  int error = _speedRamp.value() - deltaTicks;
  float i = _iAcc + (_loopTime * error);
  float d = (error - _lastError) / _loopTime;
  int ticksSpeed = (Kp * error) + (Ki * i) + (Kd * d);

  // Serial.print(_goalSpeed);
  // Serial.print("\t");
  // Serial.print(_speedRamp.value());
  // Serial.print("\t");
  // Serial.print(deltaTicks);
  // Serial.print("\t");
  // Serial.print(error);
  // Serial.print("\t");
  // Serial.print(ticksSpeed);
  // Serial.println(" ");

  _numLastTicks = _numTicks;
  _lastError = error;
  _iAcc = i;

  int pwmSpeed = constrain(ticksToPwm(ticksSpeed, _maxSpeed), 0, 255);

  if (_hardStop) {
    deltaTicks = 0;
    pwmSpeed = 0;
  }

  motorControl(
    _direction == 1 ? pwmSpeed : 0,
    _direction == -1 ? pwmSpeed : 0
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
