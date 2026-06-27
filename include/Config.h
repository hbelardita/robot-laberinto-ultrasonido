#pragma once

// =============================================================================
// Config.h — Central configuration for maze-solving robot
//
// COMPETITION RULES embedded here:
//   - 15 s immobile → penalty; STUCK_TIMEOUT_MS = 12 000 ms (3 s safety margin)
//   - Exit cell = white floor detected by IR sensor
//   - Robot must be fully autonomous; no external comms during run
//
// ALL calibration-sensitive values are in this file ONLY.
// Never hardcode these values in logic files — change them here and recompile.
// =============================================================================

// ---------------------------------------------------------------------------
// Hardware Test / Debug Mode
// Set to true to disable maze solver and enable Serial commands for testing:
// w/a/s/d = move, x = stop, c/l/r = scan center/left/right, i = read IR
// ---------------------------------------------------------------------------
constexpr bool HARDWARE_TEST_MODE = false;

// ---------------------------------------------------------------------------
// Pin assignments — Arduino Uno
// Keep PWM pins on hardware-PWM capable pins: 3,5,6,9,10,11
// ---------------------------------------------------------------------------

// L298N motor driver
// Motor A = left wheel,  Motor B = right wheel
constexpr uint8_t PIN_IN1  = 4;   // Swapped (was 2)
constexpr uint8_t PIN_IN2  = 10;  // Swapped (was 9)
constexpr uint8_t PIN_IN3  = 2;   // Swapped (was 4)
constexpr uint8_t PIN_IN4  = 9;   // Swapped (was 10)
constexpr uint8_t PIN_ENA  = 5;   // Swapped (was 3)
constexpr uint8_t PIN_ENB  = 3;   // Swapped (was 5)

// SG90 servo — carries the HC-SR04
constexpr uint8_t PIN_SERVO = 6;  // PWM-capable (Timer0) — Servo lib handles timing

// HC-SR04 ultrasonic sensor (mounted on servo)
constexpr uint8_t PIN_TRIG = 7;
constexpr uint8_t PIN_ECHO = 8;

// IR floor sensor (TCRT5000 / FC-51) — digital output, LOW = white (reflective)
// Using digital pin 11 with threshold logic; module's onboard comparator gives D0 output
constexpr uint8_t PIN_IR_FLOOR = 11;

// Built-in status LED
constexpr uint8_t PIN_LED = 13;

// ---------------------------------------------------------------------------
// Motor speed — PWM values (0–255)
// Calibrate: 130 gives ~30% of theoretical max; safe for narrow corridors
// ---------------------------------------------------------------------------
constexpr uint8_t MOTOR_SPEED  = 130;  // Cruise speed — straight movement
constexpr uint8_t TURN_SPEED   = 180;  // Turn speed — increased for more power

// ---------------------------------------------------------------------------
// Progressive braking — prevents wall collisions during advance
// As the robot approaches a wall, speed decreases linearly from MOTOR_SPEED
// to MOTOR_SPEED_SLOW. Below EMERGENCY_STOP_CM, immediate hard stop.
// ---------------------------------------------------------------------------
constexpr uint8_t MOTOR_SPEED_SLOW    = 80;   // Minimum speed during braking zone
constexpr uint8_t BRAKE_START_CM      = 20;   // Start slowing down at this front distance
constexpr uint8_t EMERGENCY_STOP_CM   = 8;    // Hard stop — too close to wall

// ---------------------------------------------------------------------------
// Servo angles (SG90: 0°–180°)
// Physical mounting: servo horn forward, sensor facing front of robot
//   160° → sensor points left of robot heading
//    90° → sensor points straight ahead
//    20° → sensor points right of robot heading
// If readings feel mirrored, swap LEFT_DEG and RIGHT_DEG
// ---------------------------------------------------------------------------
constexpr uint8_t SERVO_LEFT_DEG   = 160;
constexpr uint8_t SERVO_CENTER_DEG = 90;
constexpr uint8_t SERVO_RIGHT_DEG  = 20;

// Time to wait after servo moves before taking a measurement (ms)
// SG90 angular speed ~60°/0.1 s → 70° travel ≈ 117 ms; 300 ms adds safety margin
constexpr uint16_t SERVO_SETTLE_MS = 300;

// ---------------------------------------------------------------------------
// Distance thresholds (cm) — tuned for 25 cm cell width
// ---------------------------------------------------------------------------

// Closer than this → wall is present in that direction
// 15 cm chosen to safely fit within half a cell width (12.5 cm) with margin
constexpr uint8_t WALL_DISTANCE_THRESHOLD_CM = 15;

// Nominal maze cell width (cm)
constexpr uint8_t CELL_WIDTH_CM = 25;

// Distance to travel when advancing one cell.
// Strategy: measure front wall before moving. Drive forward until front wall
// reading decreases by ≈ADVANCE_TARGET_CM or a new wall appears within threshold.
// Set slightly less than CELL_WIDTH_CM to avoid bumping the far wall.
constexpr uint8_t ADVANCE_TARGET_CM = 18;

// ---------------------------------------------------------------------------
// Timing constants (ms) — calibrate on actual hardware
// ---------------------------------------------------------------------------

// Time for a 90° pivot turn at TURN_SPEED.
// Calibration guide: run turnLeft/Right for N ms on flat surface, measure angle.
// Typical range 450–650 ms depending on battery charge and floor friction.
constexpr uint16_t TURN_TIME_MS = 850;

// Post-turn sensor verification tolerance (cm)
// After a turn, re-scan center. Expected distance should change by at least this
// amount compared to pre-turn reading (validates turn actually happened).
constexpr uint8_t TURN_VERIFY_TOLERANCE_CM = 5;

// ---------------------------------------------------------------------------
// IR floor sensor — exit detection
// Using digital pin: LOW = white floor detected (module pulls D0 low on reflection)
// This constant documents the logic level; actual check uses digitalRead == LOW
// ---------------------------------------------------------------------------
constexpr uint16_t IR_EXIT_THRESHOLD = 500;  // Retained for analog fallback reference

// ---------------------------------------------------------------------------
// Competition safety — immobility timeout
// Rule: 15 s without movement = penalty
// We stop and signal error at 12 s to stay 3 s inside the rule
// ---------------------------------------------------------------------------
constexpr uint32_t STUCK_TIMEOUT_MS = 12000UL;

// ---------------------------------------------------------------------------
// Miscellaneous
// ---------------------------------------------------------------------------
constexpr uint16_t SERIAL_BAUD = 9600;

// Maximum valid HC-SR04 reading — beyond this the echo is noise
constexpr uint16_t HC_SR04_MAX_DISTANCE_CM = 400;

// How many HC-SR04 samples to take per measurement (median filter)
constexpr uint8_t HC_SR04_SAMPLES = 3;

// Advance fallback timeout: if front wall never appears, stop after this many ms.
// Calculated: ADVANCE_TARGET_CM / avg robot speed. Tune if robot overshoots.
constexpr uint16_t ADVANCE_TIMEOUT_MS = 1000;

// Delay between consecutive HC-SR04 pulses to avoid echo contamination (ms)
constexpr uint8_t HC_SR04_PULSE_GAP_MS = 30;
