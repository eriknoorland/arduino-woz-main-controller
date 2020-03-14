#ifndef MotorController_h
#define MotorController_h

#include "Arduino.h"
#include "Ramp.h"

class MotorController {
  public:
    MotorController(int maxSpeed, float numTicksPerRevolution, float wheelCircumference, int loopTime);
    void setup(int enable, int enableB, int pwm1, int pwm2, int encoderA, int encoderB);
    void loop();
    void move(int speed, int direction, int accelerationDuration);
    void stop(bool hard, int decelerationDuration);
    void changeSpeed(int speed, int decelerationDuration);
    void correctSpeed(int speed);
    int getSpeed();
    void onEncoderTick();
    int onTimerInterrupt();

  private:
    int _pwm1Pin;
    int _pwm2Pin;
    int _maxSpeed;
    float _numTicksPerRevolution;
    float _wheelCircumference;
    int _loopTime;
    int _goalSpeed;
    int _direction;
    float _lastError;
    float _iAcc;
    bool _hardStop;
    unsigned long _numTicks;
    unsigned long _numLastTicks;
    int _tickSpeed;
    ramp _speedRamp;
    void motorControl(int pwm1, int pwm2);
    int ticksToPwm(int tickSpeed, int maxSpeed);
};

#endif
