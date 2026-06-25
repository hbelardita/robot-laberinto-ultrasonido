// =============================================================================
// main.cpp — Maze-solving robot entry point
//
// Hardware summary:
//   Arduino Uno (ATmega328P, 16 MHz, 32 KB flash, 2 KB SRAM)
//   L298N dual H-bridge motor driver
//   SG90 servo + HC-SR04 ultrasonic sensor (front scanner)
//   TCRT5000 / FC-51 IR floor sensor (exit detection)
//   2× 18650 in series → ~7.4 V supply
//
// Competition constraints (baked into constants in Config.h):
//   - 15 s immobile → penalty  (STUCK_TIMEOUT_MS = 12 000 ms)
//   - Exit = white floor cell  (IR sensor, PIN_IR_FLOOR)
//   - Fully autonomous         (no external comms during run)
//
// Program flow:
//   setup() → initializes all hardware, waits for sensor settle,
//             blinks LED 3× as "ready" signal, then 2 s countdown.
//   loop()  → calls solver.step() each iteration.
//             When exit is found, stops and blinks rapidly forever.
// =============================================================================

#include <Arduino.h>
#include "Config.h"
#include "MotorDriver.h"
#include "UltrasonicScanner.h"
#include "MazeSolver.h"

// ---------------------------------------------------------------------------
// Global objects — static allocation only (no heap on AVR)
// ---------------------------------------------------------------------------
MotorDriver       motor;
UltrasonicScanner scanner;
MazeSolver        solver(motor, scanner);  // Dependency injection via references

// ---------------------------------------------------------------------------
// Helper: blink the status LED a given number of times
// ---------------------------------------------------------------------------
static void blinkLed(uint8_t times, uint16_t onMs, uint16_t offMs) {
    for (uint8_t i = 0; i < times; i++) {
        digitalWrite(PIN_LED, HIGH);
        delay(onMs);
        digitalWrite(PIN_LED, LOW);
        delay(offMs);
    }
}

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
void setup() {
    // Serial first — allows us to log initialization problems
    Serial.begin(SERIAL_BAUD);
    Serial.println(F("=== Maze Solver Boot ==="));
    Serial.print(F("Config: MOTOR_SPEED="));   Serial.println(MOTOR_SPEED);
    Serial.print(F("        TURN_SPEED="));     Serial.println(TURN_SPEED);
    Serial.print(F("        TURN_TIME_MS="));   Serial.println(TURN_TIME_MS);
    Serial.print(F("        WALL_THRESH="));    Serial.print(WALL_DISTANCE_THRESHOLD_CM);
    Serial.println(F("cm"));
    Serial.print(F("        ADVANCE_TGT="));    Serial.print(ADVANCE_TARGET_CM);
    Serial.println(F("cm"));

    // Status LED
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);

    // Initialize subsystems in dependency order
    motor.begin();
    Serial.println(F("[INIT] MotorDriver OK"));

    scanner.begin();  // Also centers servo and settles
    Serial.println(F("[INIT] UltrasonicScanner OK"));

    solver.begin();
    Serial.println(F("[INIT] MazeSolver OK"));

    // Allow HC-SR04 and servo to fully settle after power-on
    // (HC-SR04 datasheet recommends >60 ms after power before first reading)
    delay(500);

    // "Ready" signal: 3 slow LED blinks visible to the operator
    Serial.println(F("[INIT] Ready — 3 blinks, then 2 s countdown"));
    blinkLed(3, 300, 300);

    // 2-second delay: operator can step away, robot won't move yet
    // This also prevents false starts if the robot is still being placed
    delay(2000);

    if (HARDWARE_TEST_MODE) {
        Serial.println(F("\n=== TEST MODE ACTIVE ==="));
        Serial.println(F("Send via Serial:"));
        Serial.println(F("  w: Forward   s: Backward   a: Left   d: Right"));
        Serial.println(F("  q: 90° Left  e: 90° Right  u: 180° U-Turn"));
        Serial.println(F("  x: Stop Motors"));
        Serial.println(F("  c: Scan Center   l: Scan Left   r: Scan Right"));
        Serial.println(F("  i: Read IR Floor Sensor"));
        Serial.println(F("========================\n"));
    } else {
        Serial.println(F("[RUN] Starting solver..."));
    }
}

// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------
void loop() {
    if (HARDWARE_TEST_MODE) {
        if (Serial.available() > 0) {
            char cmd = Serial.read();
            switch (cmd) {
                case 'w': Serial.println(F("Motor: FWD")); motor.forward(); break;
                case 's': Serial.println(F("Motor: REV")); motor.backward(); break;
                case 'a': Serial.println(F("Motor: LEFT")); motor.turnLeft(); break;
                case 'd': Serial.println(F("Motor: RIGHT")); motor.turnRight(); break;
                case 'q':
                    Serial.println(F("Test: 90 L")); motor.turnLeft();
                    delay(TURN_TIME_MS); motor.stop(); break;
                case 'e':
                    Serial.println(F("Test: 90 R")); motor.turnRight();
                    delay(TURN_TIME_MS); motor.stop(); break;
                case 'u':
                    Serial.println(F("Test: 180 U-Turn"));
                    motor.turnRight(); delay(TURN_TIME_MS); motor.stop(); delay(150);
                    motor.turnRight(); delay(TURN_TIME_MS); motor.stop(); break;
                case 'x': Serial.println(F("Motor: STOP")); motor.stop(); break;
                case 'c':
                    Serial.print(F("Scan CENTER: "));
                    Serial.print(scanner.scanCenter()); Serial.println(F(" cm"));
                    break;
                case 'l':
                    Serial.print(F("Scan LEFT: "));
                    Serial.print(scanner.scanLeft()); Serial.println(F(" cm"));
                    break;
                case 'r':
                    Serial.print(F("Scan RIGHT: "));
                    Serial.print(scanner.scanRight()); Serial.println(F(" cm"));
                    break;
                case 'i':
                    Serial.print(F("IR Sensor: "));
                    Serial.println(digitalRead(PIN_IR_FLOOR) == LOW ? F("WHITE (0)") : F("BLACK (1)"));
                    break;
            }
        }

        // static unsigned long lastPrint = 0;
        // if (millis() - lastPrint > 500) {
        //     Serial.print(F("Dist Centro: "));
        //     Serial.print(scanner.scanCenter());
        //     Serial.println(F(" cm"));
        //     lastPrint = millis();
        // }
        return;
    }

    if (solver.isExitFound()) {
        // Exit has been found — blink rapidly to signal completion
        // Motors are already stopped inside handleExit()
        blinkLed(1, 100, 100);  // Rapid blink: 5 Hz flash
        return;
    }

    if (solver.getState() == SolverState::ERROR) {
        // Stuck timeout: slow SOS-style blink (3 fast, 3 slow, 3 fast)
        // This pattern is recognizable on the competition floor
        blinkLed(3, 100, 100);  // 3 fast
        delay(300);
        blinkLed(3, 400, 100);  // 3 slow
        delay(300);
        blinkLed(3, 100, 100);  // 3 fast
        delay(1000);
        return;
    }

    // Normal operation: execute one solver step
    // Each step() call is one full decision + action cycle
    solver.step();

    // Brief yield between steps — ensures Serial TX buffer can flush between
    // dense scan+log cycles, and gives the motor driver time to react.
    // Not strictly required but improves Serial readability during debugging.
    delay(50);
}
