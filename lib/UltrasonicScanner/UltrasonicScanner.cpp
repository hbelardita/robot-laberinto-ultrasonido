#include "UltrasonicScanner.h"

// =============================================================================
// UltrasonicScanner.cpp — HC-SR04 + SG90 servo implementation
//
// HC-SR04 timing constants (datasheet):
//   Trigger pulse : 10 µs minimum HIGH
//   Echo timeout  : ~38 ms (no object) — pulseIn default timeout is 1 s,
//                   we cap it at MAX_DISTANCE to avoid blocking the loop.
//
// Speed of sound: 343 m/s at 20°C → 0.0343 cm/µs
// Distance formula: d = (echo_us * 0.0343) / 2
//                     = echo_us / 58.23  ≈ echo_us / 58
// We use 58.0f as the divisor (commonly used, matches datasheet example).
// ---------------------------------------------------------------------------
// Calibration note: at temperatures far from 20°C, distance accuracy degrades
// (~0.17% per °C). For indoor competition use this is negligible.
// =============================================================================

// Speed of sound divisor (µs → cm) — one-way half of round trip
static const float SOUND_DIVISOR = 58.0f;

// Maximum echo pulse width to accept (µs). Beyond this = no valid echo.
// 400 cm * 58 µs/cm = 23 200 µs ≈ 23 ms. Add small margin.
static const uint32_t ECHO_TIMEOUT_US = 24000UL;

// ---------------------------------------------------------------------------
// Initialization
// ---------------------------------------------------------------------------

void UltrasonicScanner::begin() {
    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT);
    digitalWrite(PIN_TRIG, LOW);  // Ensure TRIG starts LOW

    _servo.attach(PIN_SERVO);
    // Center servo immediately so the sensor faces forward before the run starts
    moveServo(SERVO_CENTER_DEG);
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void UltrasonicScanner::moveServo(uint8_t degrees) {
    _servo.write(degrees);
    delay(SERVO_SETTLE_MS);  // Must be blocking here — sensor needs mechanical rest
}

float UltrasonicScanner::rawReadCm() {
    // Ensure TRIG is LOW before pulse (some modules need a clean leading edge)
    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(2);

    // Send the required 10 µs trigger pulse
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);

    // Measure echo pulse width — HIGH state duration on ECHO pin
    // pulseIn blocks until the pulse ends or timeout expires
    uint32_t echoDuration = pulseIn(PIN_ECHO, HIGH, ECHO_TIMEOUT_US);

    if (echoDuration == 0) {
        // Timeout: no echo received — treat as maximum range (open space)
        return (float)HC_SR04_MAX_DISTANCE_CM;
    }

    float distanceCm = echoDuration / SOUND_DIVISOR;

    // Clamp to valid sensor range — values below 2 cm are sensor blind spot
    if (distanceCm < 2.0f) return 2.0f;
    if (distanceCm > HC_SR04_MAX_DISTANCE_CM) return (float)HC_SR04_MAX_DISTANCE_CM;

    return distanceCm;
}

void UltrasonicScanner::sort3(float arr[3]) {
    // Insertion sort for exactly 3 elements — minimal code size on AVR
    for (uint8_t i = 1; i < 3; i++) {
        float key = arr[i];
        int8_t j = i - 1;
        while (j >= 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            j--;
        }
        arr[j + 1] = key;
    }
}

// ---------------------------------------------------------------------------
// Public measurement
// ---------------------------------------------------------------------------

float UltrasonicScanner::measureDistanceCm() {
    float samples[HC_SR04_SAMPLES];

    for (uint8_t i = 0; i < HC_SR04_SAMPLES; i++) {
        samples[i] = rawReadCm();
        // Small gap between pulses prevents the outgoing pulse from triggering
        // the next measurement's echo receiver (echo contamination)
        if (i < HC_SR04_SAMPLES - 1) {
            delay(HC_SR04_PULSE_GAP_MS);
        }
    }

    sort3(samples);
    // Return the median (middle value) — robust against single outliers
    return samples[HC_SR04_SAMPLES / 2];
}

// ---------------------------------------------------------------------------
// Directional scans
// ---------------------------------------------------------------------------

float UltrasonicScanner::scanLeft() {
    moveServo(SERVO_LEFT_DEG);
    return measureDistanceCm();
}

float UltrasonicScanner::scanCenter() {
    moveServo(SERVO_CENTER_DEG);
    return measureDistanceCm();
}

float UltrasonicScanner::scanRight() {
    moveServo(SERVO_RIGHT_DEG);
    return measureDistanceCm();
}

void UltrasonicScanner::scanAll(float &left, float &center, float &right) {
    // Scan order: left → center → right
    // Left first because the Left-Hand Rule checks it first — avoids an extra
    // servo sweep when the left passage is open (the common case).
    left   = scanLeft();
    center = scanCenter();
    right  = scanRight();

    // Leave servo at center so the robot can travel forward safely
    // (center scan already moved it there — this is just a documentation reminder)
}

// ---------------------------------------------------------------------------
// Wall detection
// ---------------------------------------------------------------------------

bool UltrasonicScanner::isWall(float distanceCm) {
    // A reading below the threshold reliably indicates a wall at maze-corridor scale
    return (distanceCm < (float)WALL_DISTANCE_THRESHOLD_CM);
}
