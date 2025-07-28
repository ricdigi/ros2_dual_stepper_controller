/*
  ENCODER ACCURACY – SEPARATE FORWARD‑ONLY / BACKWARD‑ONLY RUNS
  -------------------------------------------------------------
  • 10 × +360 °  (forward sequence)
  • Return to zero
  • 10 × –360 °  (backward sequence)
  Each move prints start, end, error.
*/

#include <Arduino.h>
#include <Wire.h>
#include <AccelStepper.h>
#include "MagneticEncoder.h"

// ── pin map ────────────────────────────────────────────────────────────────
#define STEP_PIN_A      15
#define STEP_DIR_PIN_A  21
#define ENABLE_PIN_AB   14   // LOW = enable

#define MUX_PIN_A       27
#define MUX_PIN_B       28
#define DIR_PIN_A       29
#define DIR_PIN_B       30
// ───────────────────────────────────────────────────────────────────────────

constexpr int   FULL_STEPS   = 200;
constexpr int   MICROSTEP    = 16;
constexpr long  STEPS_REV    = FULL_STEPS * MICROSTEP;
constexpr int   N_MOVES      = 10;

constexpr float MAX_RPM      = 60.0f;   // safe speed
constexpr float DEG_REV      = 360.0f;

AccelStepper    motor(AccelStepper::DRIVER, STEP_PIN_A, STEP_DIR_PIN_A);
MagneticEncoder enc;

// ── helpers ────────────────────────────────────────────────────────────────
static inline float wrapDeg(float d)
{
  while (d >  180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

void moveAndReport(long steps, const char* tag, int idx)
{
  const float ideal = (steps > 0 ?  DEG_REV : -DEG_REV);
  float startDeg = enc.getEncoderAData();

  motor.move(steps);
  while (motor.distanceToGo() != 0) {
    motor.run();
    enc.readSensors();
  }
  delay(200);                           // encoder settle
  float endDeg = enc.getEncoderAData();

  float err = wrapDeg(endDeg - startDeg - ideal);

  Serial.print(idx); Serial.print(','); Serial.print(tag); Serial.print(',');
  Serial.print(startDeg, 2); Serial.print(','); Serial.print(endDeg, 2);
  Serial.print(','); Serial.println(err, 2);
}

void gotoZero()
{
  long backSteps = -(motor.currentPosition() % STEPS_REV);
  motor.move(backSteps);
  while (motor.distanceToGo() != 0) motor.run();
  delay(300);
}

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  pinMode(ENABLE_PIN_AB, OUTPUT);
  digitalWrite(ENABLE_PIN_AB, LOW);      // enable driver

  motor.setMaxSpeed((MAX_RPM / 60.0f) * STEPS_REV);
  motor.setAcceleration(motor.maxSpeed() * 4);

  enc.encoderInit(MUX_PIN_A, MUX_PIN_B, DIR_PIN_A, DIR_PIN_B);

  Serial.println("idx,dir,start_deg,end_deg,error_deg");

  // ── forward‑only test ─────────────────────────────────────────
  for (int i = 0; i < N_MOVES; ++i)
    moveAndReport(+STEPS_REV, "fwd", i);

  // return to zero before backward run
  gotoZero();

  // ── backward‑only test ────────────────────────────────────────
  for (int i = 0; i < N_MOVES; ++i)
    moveAndReport(-STEPS_REV, "rev", i);

  Serial.println("Test complete. Halting.");
  while (true) {}                        // stop
}

void loop() {}   // never reached
