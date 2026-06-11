#pragma once

#include <Arduino.h>
#include "Config.h"

// =============================================================================
// MotorDriver.h — Interface for dual DC motor control via L298N
//
// Wiring convention:
//   Motor A = LEFT  wheel  (IN1, IN2, ENA)
//   Motor B = RIGHT wheel  (IN3, IN4, ENB)
//
// L298N truth table per channel:
//   INx=HIGH, INy=LOW  → forward
//   INx=LOW,  INy=HIGH → backward
//   INx=LOW,  INy=LOW  → coast (free spin)
//   INx=HIGH, INy=HIGH → brake (short circuit — avoid)
//
// All speed parameters are PWM values 0–255. Actual constants come from Config.h.
// =============================================================================

class MotorDriver {
public:
    // Initialize all motor control pins. Call once in setup().
    void begin();

    // Drive both motors forward at the given PWM speed.
    void forward(uint8_t speed = MOTOR_SPEED);

    // Drive both motors backward at the given PWM speed.
    void backward(uint8_t speed = MOTOR_SPEED);

    // Pivot turn LEFT:  left motor backward, right motor forward.
    // Results in an in-place rotation to the left.
    void turnLeft(uint8_t speed = TURN_SPEED);

    // Pivot turn RIGHT: left motor forward, right motor backward.
    // Results in an in-place rotation to the right.
    void turnRight(uint8_t speed = TURN_SPEED);

    // Active brake — both motors set to coast (ENA/ENB = 0).
    // For hard braking set IN pins to same state; here we use coast + PWM=0
    // which is gentler and avoids motor driver heating.
    void stop();

    // Gentle correction: reduce LEFT motor speed to drift slightly left.
    // Used when the robot is veering right inside a corridor.
    void adjustLeft(uint8_t speed = MOTOR_SPEED);

    // Gentle correction: reduce RIGHT motor speed to drift slightly right.
    // Used when the robot is veering left inside a corridor.
    void adjustRight(uint8_t speed = MOTOR_SPEED);

private:
    // Set Motor A (left) direction and speed.
    //   dir=true  → forward
    //   dir=false → backward
    void setMotorA(bool forward, uint8_t speed);

    // Set Motor B (right) direction and speed.
    void setMotorB(bool forward, uint8_t speed);
};
