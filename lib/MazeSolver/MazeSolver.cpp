#include "MazeSolver.h"

// =============================================================================
// MazeSolver.cpp — Left-Hand Rule implementation
//
// Step() execution flow per call:
//   1. Check IR exit floor sensor          → EXIT_FOUND if triggered
//   2. Check stuck timeout                 → ERROR if exceeded
//   3. Scan all three directions           → left, center, right distances
//   4. Apply Left-Hand Rule priority       → choose action
//   5. Execute chosen action               → turn or advance
//   6. Update step counter & timestamp
//
// WHY distance-based advancement?
//   Time-based advancement accumulates error over multiple cells (battery
//   voltage drops, friction changes). Measuring the front wall distance and
//   stopping when it decreases by ADVANCE_TARGET_CM gives cell-accurate
//   positioning without encoders.
//
// WHY a time-based fallback for advancement?
//   The last cell before the exit has a white floor and no front wall
//   (the exit is detected by the IR sensor). Without the time fallback
//   the robot would drive forward indefinitely. ADVANCE_TIMEOUT_MS caps this.
// =============================================================================

// ---------------------------------------------------------------------------
// Constructor & init
// ---------------------------------------------------------------------------

MazeSolver::MazeSolver(MotorDriver &motor, UltrasonicScanner &scanner)
    : _motor(motor), _scanner(scanner),
      _state(SolverState::SCANNING),
      _stepCount(0),
      _lastMovementMs(0),
      _turnAroundPhase(0)
{}

void MazeSolver::begin() {
    pinMode(PIN_IR_FLOOR, INPUT);
    _state           = SolverState::SCANNING;
    _stepCount       = 0;
    _lastMovementMs  = millis();
    _turnAroundPhase = 0;

    Serial.print(F("[MazeSolver] begin — Left-Hand Rule active. STUCK_TIMEOUT="));
    Serial.print(STUCK_TIMEOUT_MS);
    Serial.println(F("ms"));
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

void MazeSolver::step() {
    _stepCount++;

    // --- 1. Exit detection (highest priority — check before any movement) ---
    if (isExitFloor()) {
        handleExit();
        return;
    }

    // --- 2. Stuck timeout guard ---
    // Competition rule: 15 s immobile = penalty.
    // We trigger at STUCK_TIMEOUT_MS (12 s) to stay safely inside the limit.
    if ((millis() - _lastMovementMs) > STUCK_TIMEOUT_MS) {
        handleStuck();
        return;
    }

    // --- 3. Error/Exit states: do nothing (caller checks isExitFound()) ---
    if (_state == SolverState::EXIT_FOUND || _state == SolverState::ERROR) {
        return;
    }

    // --- 4. Scan all directions ---
    float distLeft, distCenter, distRight;
    _scanner.scanAll(distLeft, distCenter, distRight);

    bool wallLeft   = _scanner.isWall(distLeft);
    bool wallFront  = _scanner.isWall(distCenter);
    bool wallRight  = _scanner.isWall(distRight);

    // --- 5. Left-Hand Rule decision ---
    const char *decision = "";

    if (!wallLeft) {
        // Left passage open → turn left and advance (maintain left wall contact)
        decision = "TURN_LEFT";
        logDecision(distLeft, distCenter, distRight, decision);
        executeTurnLeft();
        advanceOneCell();

    } else if (!wallFront) {
        // Left blocked, front open → advance straight
        decision = "ADVANCE";
        logDecision(distLeft, distCenter, distRight, decision);
        advanceOneCell();

    } else if (!wallRight) {
        // Left and front blocked, right open → turn right and advance
        decision = "TURN_RIGHT";
        logDecision(distLeft, distCenter, distRight, decision);
        executeTurnRight();
        advanceOneCell();

    } else {
        // Dead end: all three directions blocked → U-turn (180°)
        decision = "TURN_AROUND";
        logDecision(distLeft, distCenter, distRight, decision);
        executeTurnAround();
        // After U-turn, do NOT advance — let next step() re-scan and decide
    }

    // Update last-movement timestamp (any decision counts as activity)
    _lastMovementMs = millis();
    _state = SolverState::SCANNING;
}

bool MazeSolver::isExitFound() const {
    return (_state == SolverState::EXIT_FOUND);
}

uint32_t MazeSolver::getStepCount() const {
    return _stepCount;
}

SolverState MazeSolver::getState() const {
    return _state;
}

// ---------------------------------------------------------------------------
// Private: exit detection
// ---------------------------------------------------------------------------

bool MazeSolver::isExitFloor() {
    // FC-51 / TCRT5000 module: D0 output goes LOW when it detects a reflective
    // (white) surface. We read digital so we don't need analog threshold math.
    // IR_EXIT_THRESHOLD in Config.h is retained for analog fallback reference.
    return (digitalRead(PIN_IR_FLOOR) == LOW);
}

// ---------------------------------------------------------------------------
// Private: advancement (distance-based + time fallback)
// ---------------------------------------------------------------------------

void MazeSolver::advanceOneCell() {
    _state = SolverState::ADVANCING;

    // Sample front distance BEFORE moving to establish baseline.
    // We drive until the front wall is within WALL_DISTANCE_THRESHOLD_CM
    // OR until we've waited ADVANCE_TIMEOUT_MS (open corridor / last cell).
    float startDist = _scanner.scanCenter();

    Serial.print(F("[ADV] startDist="));
    Serial.print(startDist);
    Serial.println(F("cm — driving forward"));

    _motor.forward(MOTOR_SPEED);

    uint32_t advStart = millis();
    bool cellCrossed  = false;

    while (!cellCrossed) {
        // Check IR sensor in mid-advance — catch the exit cell even while moving
        if (isExitFloor()) {
            _motor.stop();
            handleExit();
            return;
        }

        // Check stuck timeout even mid-advance
        if ((millis() - _lastMovementMs) > STUCK_TIMEOUT_MS) {
            _motor.stop();
            handleStuck();
            return;
        }

        uint32_t elapsed = millis() - advStart;

        // Condition A: front wall now within threshold — we're at the next cell
        // (This is the primary stopping condition — distance-based)
        float currentFront = _scanner.scanCenter();
        if (_scanner.isWall(currentFront)) {
            Serial.print(F("[ADV] wall detected at "));
            Serial.print(currentFront);
            Serial.println(F("cm — stopping"));
            cellCrossed = true;
            break;
        }

        // Condition B: fallback timeout — no wall ever appeared (open far end)
        // This handles the exit cell and very long corridors beyond sensor range
        if (elapsed >= ADVANCE_TIMEOUT_MS) {
            Serial.println(F("[ADV] timeout — assuming cell crossed"));
            cellCrossed = true;
            break;
        }

        // Small yield — each loop iteration includes a sensor read (~60ms with
        // median filter). This is fast enough for corridor-speed movement.
    }

    _motor.stop();
    // Brief settle delay — allows the robot body to stop oscillating before next scan
    delay(150);

    Serial.println(F("[ADV] stopped"));
}

// ---------------------------------------------------------------------------
// Private: turn execution
// ---------------------------------------------------------------------------

void MazeSolver::executeTurnLeft() {
    _state = SolverState::TURNING_LEFT;
    Serial.println(F("[TURN] LEFT"));

    _motor.turnLeft(TURN_SPEED);
    delay(TURN_TIME_MS);  // Blocking: TURN_TIME_MS is short and accuracy matters
    _motor.stop();
    delay(100);  // Let inertia settle before re-scanning

    verifyAfterTurn("LEFT");
}

void MazeSolver::executeTurnRight() {
    _state = SolverState::TURNING_RIGHT;
    Serial.println(F("[TURN] RIGHT"));

    _motor.turnRight(TURN_SPEED);
    delay(TURN_TIME_MS);
    _motor.stop();
    delay(100);

    verifyAfterTurn("RIGHT");
}

void MazeSolver::executeTurnAround() {
    _state = SolverState::TURNING_AROUND;
    Serial.println(F("[TURN] AROUND — two sequential right pivots"));

    // First 90° right
    _motor.turnRight(TURN_SPEED);
    delay(TURN_TIME_MS);
    _motor.stop();
    delay(150);  // Slightly longer settle between the two turns

    // Second 90° right → total 180°
    _motor.turnRight(TURN_SPEED);
    delay(TURN_TIME_MS);
    _motor.stop();
    delay(150);

    verifyAfterTurn("AROUND");
}

float MazeSolver::verifyAfterTurn(const char *dirLabel) {
    // Post-turn verification: scan center and log.
    // We do not abort if the reading is unexpected — the next step() will re-scan
    // and correct. Verification is for debugging / competition tuning only.
    float frontDist = _scanner.scanCenter();

    Serial.print(F("[VERIFY] after "));
    Serial.print(dirLabel);
    Serial.print(F(" turn — front dist = "));
    Serial.print(frontDist);
    Serial.print(F("cm → "));
    Serial.println(_scanner.isWall(frontDist) ? F("WALL") : F("OPEN"));

    return frontDist;
}

// ---------------------------------------------------------------------------
// Private: terminal conditions
// ---------------------------------------------------------------------------

void MazeSolver::handleExit() {
    _motor.stop();
    _state = SolverState::EXIT_FOUND;

    Serial.print(F("[EXIT] Found! Steps taken: "));
    Serial.println(_stepCount);
}

void MazeSolver::handleStuck() {
    _motor.stop();
    _state = SolverState::ERROR;

    Serial.print(F("[ERROR] Stuck timeout ("));
    Serial.print(STUCK_TIMEOUT_MS);
    Serial.print(F("ms) exceeded at step "));
    Serial.println(_stepCount);
    // Competition note: at this point the run is likely penalized.
    // Hardware reset or manual restart required.
}

// ---------------------------------------------------------------------------
// Private: serial debug helper
// ---------------------------------------------------------------------------

void MazeSolver::logDecision(float left, float center, float right, const char *decision) {
    Serial.print(F("[STEP "));
    Serial.print(_stepCount);
    Serial.print(F("] L="));
    Serial.print(left, 1);
    Serial.print(F("cm C="));
    Serial.print(center, 1);
    Serial.print(F("cm R="));
    Serial.print(right, 1);
    Serial.print(F("cm → "));
    Serial.println(decision);
}
