#pragma once

#include <Arduino.h>
#include <Servo.h>
#include "Config.h"

// =============================================================================
// UltrasonicScanner.h — Servo-mounted HC-SR04 sensor interface
//
// The servo sweeps to three positions (left / center / right) allowing the
// robot to assess walls in all three forward-facing directions with a single
// sensor. This is the standard approach for single-sensor maze robots.
//
// HC-SR04 timing:
//   - Trigger: 10 µs HIGH pulse on TRIG pin
//   - Echo:    HIGH pulse on ECHO pin whose width = round-trip time in µs
//   - Distance = (pulse_µs / 2) / 29.1  →  cm
//   - Valid range: 2 cm – 400 cm; beyond 400 cm echo times out
//
// Median-of-3 filter is used to suppress occasional spurious readings
// (reflections off angled surfaces are common in indoor mazes).
// =============================================================================

class UltrasonicScanner {
public:
    // Attach servo, configure TRIG/ECHO pins, center sensor. Call once in setup().
    void begin();

    // Fire the HC-SR04 and return distance in cm.
    // Returns HC_SR04_MAX_DISTANCE_CM if no echo received (open space / too far).
    // Internally takes HC_SR04_SAMPLES readings and returns the median — avoids
    // outliers without the memory cost of a larger buffer.
    float measureDistanceCm();

    // Move servo to left position, wait for settle, measure and return distance.
    float scanLeft();

    // Move servo to center (straight ahead), wait for settle, measure and return distance.
    float scanCenter();

    // Move servo to right position, wait for settle, measure and return distance.
    float scanRight();

    // Convenience: scan all three directions in one call (left → center → right),
    // then leave servo at center for traveling.
    // Parameters are output references filled with the measured distances.
    void scanAll(float &left, float &center, float &right);

    // Returns true if the given distance indicates a wall is present.
    // Uses WALL_DISTANCE_THRESHOLD_CM from Config.h.
    bool isWall(float distanceCm);

private:
    Servo _servo;

    // Move servo to given angle and wait SERVO_SETTLE_MS.
    void moveServo(uint8_t degrees);

    // Internal single raw HC-SR04 reading (µs → cm). Returns 0 on timeout.
    float rawReadCm();

    // Sort 3 float values in-place (insertion sort — tiny, no heap needed).
    void sort3(float arr[3]);
};
