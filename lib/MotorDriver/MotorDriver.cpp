#include "MotorDriver.h"

// =============================================================================
// MotorDriver.cpp — L298N dual H-bridge implementation
//
// Pin mapping (from Config.h):
//   Motor A (LEFT):  IN1, IN2, ENA (PWM)
//   Motor B (RIGHT): IN3, IN4, ENB (PWM)
//
// Note on correction methods:
//   adjustLeft/Right reduce one motor's PWM by 30% relative to the other.
//   This creates a gentle arc — sufficient for corridor tracking.
//   If stronger correction is needed, increase the divisor.
// =============================================================================

// ---------------------------------------------------------------------------
// Initialization
// ---------------------------------------------------------------------------

void MotorDriver::begin() {
    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_IN3, OUTPUT);
    pinMode(PIN_IN4, OUTPUT);
    pinMode(PIN_ENA, OUTPUT);
    pinMode(PIN_ENB, OUTPUT);

    // Ensure motors are stopped at startup — safe default
    stop();
}

// ---------------------------------------------------------------------------
// Private helpers — low-level per-motor control
// ---------------------------------------------------------------------------

void MotorDriver::setMotorA(bool fwd, uint8_t speed) {
    // L298N: IN1=H,IN2=L → forward; IN1=L,IN2=H → backward
    digitalWrite(PIN_IN1, fwd ? HIGH : LOW);
    digitalWrite(PIN_IN2, fwd ? LOW  : HIGH);
    analogWrite(PIN_ENA, speed);
}

void MotorDriver::setMotorB(bool fwd, uint8_t speed) {
    // L298N: IN3=H,IN4=L → forward; IN3=L,IN4=H → backward
    digitalWrite(PIN_IN3, fwd ? HIGH : LOW);
    digitalWrite(PIN_IN4, fwd ? LOW  : HIGH);
    analogWrite(PIN_ENB, speed);
}

// ---------------------------------------------------------------------------
// Public movement methods
// ---------------------------------------------------------------------------

void MotorDriver::forward(uint8_t speed) {
    setMotorA(true,  speed);  // Left  → forward
    setMotorB(true,  speed);  // Right → forward
}

void MotorDriver::backward(uint8_t speed) {
    setMotorA(false, speed);  // Left  → backward
    setMotorB(false, speed);  // Right → backward
}

void MotorDriver::turnLeft(uint8_t speed) {
    // Pivot left: left wheel backward, right wheel forward
    // This keeps the robot rotating around its center axis
    setMotorA(false, speed);  // Left  → backward
    setMotorB(true,  speed);  // Right → forward
}

void MotorDriver::turnRight(uint8_t speed) {
    // Pivot right: left wheel forward, right wheel backward
    setMotorA(true,  speed);  // Left  → forward
    setMotorB(false, speed);  // Right → backward
}

void MotorDriver::stop() {
    // Coast stop: set PWM to 0 first, then release direction pins
    // Avoids a brief high-current state when changing direction
    analogWrite(PIN_ENA, 0);
    analogWrite(PIN_ENB, 0);
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, LOW);
}

void MotorDriver::adjustLeft(uint8_t speed) {
    // Reduce left motor to ~70% to arc gently left
    // "30% reduction" is a rough empirical value — tune if arcs are too tight/loose
    uint8_t reducedSpeed = (uint8_t)(speed * 0.70f);
    setMotorA(true, reducedSpeed);  // Left  → forward at reduced speed
    setMotorB(true, speed);         // Right → forward at full speed
}

void MotorDriver::adjustRight(uint8_t speed) {
    // Reduce right motor to ~70% to arc gently right
    uint8_t reducedSpeed = (uint8_t)(speed * 0.70f);
    setMotorA(true, speed);         // Left  → forward at full speed
    setMotorB(true, reducedSpeed);  // Right → forward at reduced speed
}
