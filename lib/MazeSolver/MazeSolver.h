#pragma once

#include <Arduino.h>
#include "Config.h"
#include "MotorDriver.h"
#include "UltrasonicScanner.h"

// =============================================================================
// MazeSolver.h — Left-Hand Rule maze solver
//
// Algorithm: Left-Hand Rule (wall follower)
//   Priority order at each cell decision point:
//     1. LEFT open  → turn left, advance
//     2. FRONT open → advance straight
//     3. RIGHT open → turn right, advance
//     4. All blocked → turn around (two consecutive 90° turns)
//
// Guarantees: the Left-Hand Rule guarantees finding the exit in any simply-
// connected maze (no isolated wall islands). If the maze has islands, the
// robot may loop — in competition mazes this is rarely an issue.
//
// Advancement strategy: distance-based via ultrasonic, with a time-based
// fallback (ADVANCE_TIMEOUT_MS) for cases where no front wall appears
// (e.g., corridor runs off past sensor range).
//
// Exit detection: IR floor sensor checked at the TOP of every step() call,
// before movement decisions, to catch the exit cell immediately.
// =============================================================================

// Solver state machine states
enum class SolverState : uint8_t {
    SCANNING,        // Default: deciding next direction
    ADVANCING,       // Moving forward one cell
    TURNING_LEFT,    // Executing a 90° left pivot
    TURNING_RIGHT,   // Executing a 90° right pivot
    TURNING_AROUND,  // Executing a 180° (two sequential 90° turns)
    EXIT_FOUND,      // White floor detected — run complete
    ERROR            // Stuck timeout or unrecoverable condition
};

class MazeSolver {
public:
    // Constructor: inject dependencies (no dynamic allocation)
    MazeSolver(MotorDriver &motor, UltrasonicScanner &scanner);

    // Initialize solver state. Call once in setup() after motor/scanner begin().
    void begin();

    // Execute one solver step. Call from loop().
    // Each call handles ONE decision + ONE action (advance or turn).
    // Returns immediately after action — does not block indefinitely.
    void step();

    // Returns true once the exit cell has been detected.
    bool isExitFound() const;

    // Returns total number of step() calls made (useful for serial debugging).
    uint32_t getStepCount() const;

    // Returns current solver state (useful for LED signaling / debug).
    SolverState getState() const;

private:
    MotorDriver      &_motor;
    UltrasonicScanner &_scanner;

    SolverState  _state;
    uint32_t     _stepCount;
    uint32_t     _lastMovementMs;   // millis() at last confirmed movement start
    uint8_t      _turnAroundPhase;  // 0 = first 90°, 1 = second 90° (for U-turn)

    // --- Action helpers ---

    // Check IR floor sensor — returns true if white (exit) floor detected.
    bool isExitFloor();

    // Advance one maze cell forward (distance-based with time fallback).
    // Blocks until the robot has traveled ~ADVANCE_TARGET_CM or ADVANCE_TIMEOUT_MS.
    void advanceOneCell();

    // Execute a 90° left pivot turn, then verify with sensor.
    void executeTurnLeft();

    // Execute a 90° right pivot turn, then verify with sensor.
    void executeTurnRight();

    // Execute a 180° U-turn as two sequential 90° right pivots.
    void executeTurnAround();

    // Post-turn verification: scan center, log via Serial.
    // Returns the measured front distance after turning.
    float verifyAfterTurn(const char *dirLabel);

    // Handle exit detection: stop motors, signal via Serial.
    void handleExit();

    // Handle stuck timeout: stop motors, set ERROR state, signal via Serial.
    void handleStuck();

    // Log a Serial message using F() macro to keep strings in flash (saves RAM).
    // We use a helper to avoid scattering Serial.print(F(...)) everywhere.
    void logDecision(float left, float center, float right, const char *decision);
};
