/*
 * AS5600_Cruise_Control.ino
 *
 * Reads rotational speed (RPM) from an AS5600 magnetic encoder over I2C.
 * A proportional controller compares measured RPM to a target setpoint
 * and outputs a PWM servo signal (1000–2000 µs) to drive a throttle servo.
 *
 * Hardware:
 *   - Arduino Nano (ATmega328P, new bootloader)
 *   - AS5600 encoder: SDA → A4, SCL → A5, 3.3 V or 5 V power
 *   - Servo signal output: pin D5
 *   - R/C input: pin D3 (interrupt-capable)
 *
 * Wiring note: Pull-up resistors (4.7 kΩ) on SDA/SCL are required.
 */

#include <Wire.h>
#include <Servo.h>

// ---------------------------------------------------------------------------
// AS5600 I2C constants
// ---------------------------------------------------------------------------
static const uint8_t AS5600_ADDR       = 0x36;
static const uint8_t AS5600_REG_STATUS = 0x0B;  // bit5=MH, bit4=ML, bit3=MD
static const uint8_t AS5600_REG_ANGLE  = 0x0C;  // 0x0C (high), 0x0D (low)

// ---------------------------------------------------------------------------
// Servo / PWM constants
// ---------------------------------------------------------------------------
static const uint8_t SERVO_PIN      = 5;
static const int     SERVO_MIN_US   = 1615;  // 1615 minimum throttle for driving
static const int     SERVO_MAX_US   = 1650;  // 1645 maximum throttle for driving
static const int     SERVO_NEUTRAL  = 1500;  //  neutral 

// ---------------------------------------------------------------------------
// R/C input
// ---------------------------------------------------------------------------
static const uint8_t RC_INPUT_PIN    = 3; // D3 (interrupt-capable)
static const int RC_ENABLE_MIN_US    = 1510;
static const int RC_FAST_DROP_US     = 1520;  // below this, throttle snaps to neutral instantly
static const int     RC_RPM_MIN_US   = 1520;   // RC pulse → minimum target RPM
static const int     RC_RPM_MAX_US   = 2000;   // RC pulse → maximum target RPM
static const float   TARGET_RPM_MIN  = 100.0f; // RPM at RC_RPM_MIN_US
static const float   TARGET_RPM_MAX  = 150.0f; // RPM at RC_RPM_MAX_US

// ---------------------------------------------------------------------------
// Controller tuning
// ---------------------------------------------------------------------------
float Kp             = 5.0f;   // proportional gain (µs per RPM of error)
float targetRPM      = TARGET_RPM_MIN;  // cruise setpoint (RPM), updated each loop from RC input

// ---------------------------------------------------------------------------
// Loop timing
// ---------------------------------------------------------------------------
static const uint32_t LOOP_MS = 20;   // control loop period (ms) → 50 Hz

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
Servo throttleServo;

uint16_t prevAngle      = 0;
uint32_t prevTimeMs     = 0;
float    measuredRPM    = 0.0f;
bool     controlEnabled = false;  // toggled by button

// Servo output filter
float filteredPulseUs   = SERVO_NEUTRAL;
const float FILTER_ALPHA = 1.0f; // 0.0 = no response, 1.0 = no filtering

// Slew rate limiter on servo output
// Maximum rate of change of the servo pulse width in µs per millisecond.
static const float SLEW_RATE_MAX_US_PER_MS = 0.035f;  // µs/ms — tune as needed
float slewedPulseUs = SERVO_NEUTRAL;

// R/C input state
int rcPulseUs = 1500;

// ---------------------------------------------------------------------------
// Read raw 12-bit angle from AS5600 (0–4095)
// Returns false if read fails.
// ---------------------------------------------------------------------------
bool readAngle(uint16_t &angle)
{
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_REG_ANGLE);
    if (Wire.endTransmission(false) != 0) return false;

    if (Wire.requestFrom(AS5600_ADDR, (uint8_t)2) != 2) return false;

    uint16_t high = Wire.read();
    uint16_t low  = Wire.read();
    angle = ((high & 0x0F) << 8) | low;
    return true;
}

// ---------------------------------------------------------------------------
// Check AS5600 magnet detection (MD bit must be set for valid readings)
// ---------------------------------------------------------------------------
bool magnetDetected()
{
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_REG_STATUS);
    if (Wire.endTransmission(false) != 0) return false;

    if (Wire.requestFrom(AS5600_ADDR, (uint8_t)1) != 1) return false;

    uint8_t status = Wire.read();
    return (status & 0x20) != 0;  // bit 5 = MD (magnet detected)
}

// ---------------------------------------------------------------------------
// Compute RPM from angle delta, handling 12-bit rollover
// ---------------------------------------------------------------------------
float computeRPM(uint16_t currentAngle, uint16_t previousAngle, float elapsedMs)
{
    int16_t delta = (int16_t)currentAngle - (int16_t)previousAngle;

    // Correct for 12-bit (4096-count) rollover
    if (delta >  2048) delta -= 4096;
    if (delta < -2048) delta += 4096;

    // RPM = (counts / 4096 counts-per-rev) * (60000 ms/min / elapsed ms)
    return ((float)delta / 4096.0f) * (60000.0f / elapsedMs);
}

// ---------------------------------------------------------------------------
// Map a float output value to a servo pulse width (µs), clamped to range
// ---------------------------------------------------------------------------
int outputToServoPulse(float controlOutput)
{
    int pulse = SERVO_NEUTRAL + (int)controlOutput;
    if (pulse < SERVO_MIN_US) pulse = SERVO_MIN_US;
    if (pulse > SERVO_MAX_US) pulse = SERVO_MAX_US;
    return pulse;
}

// ---------------------------------------------------------------------------
// setup
// ---------------------------------------------------------------------------
void setup()
{
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000);  // 400 kHz fast mode

    pinMode(RC_INPUT_PIN, INPUT);

    throttleServo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
    throttleServo.writeMicroseconds(SERVO_NEUTRAL);

    // Wait for magnet to be detected
    Serial.println(F("Waiting for AS5600 magnet..."));
    while (!magnetDetected()) {
        Serial.println(F("  No magnet detected. Check placement and power."));
        delay(500);
    }
    Serial.println(F("Magnet detected. Starting cruise control."));

    readAngle(prevAngle);
    prevTimeMs = millis();

    // Print CSV header for Serial Plotter / logger
    Serial.println(F("enabled,targetRPM,measuredRPM,pulseUs"));
}

// ---------------------------------------------------------------------------
// loop
// ---------------------------------------------------------------------------


void loop()
{

    // Read R/C input pulse width (in microseconds)
    int pulse = pulseIn(RC_INPUT_PIN, HIGH, 25000); // 25ms timeout
    if (pulse > 900) rcPulseUs = pulse; // ignore timeouts

    // Map RC input to target RPM (1520 µs → 100 RPM, 2000 µs → 150 RPM)
    {
        float frac = (float)(rcPulseUs - RC_RPM_MIN_US) / (float)(RC_RPM_MAX_US - RC_RPM_MIN_US);
        if (frac < 0.0f) frac = 0.0f;
        if (frac > 1.0f) frac = 1.0f;
        targetRPM = TARGET_RPM_MIN + frac * (TARGET_RPM_MAX - TARGET_RPM_MIN);
    }

    // Enable/disable control based on R/C input
    if (rcPulseUs > RC_ENABLE_MIN_US) {
        if (!controlEnabled) {
            controlEnabled = true;
            Serial.println(F("Cruise control ENABLED (RC)"));
        }
    } else {
        if (controlEnabled) {
            controlEnabled = false;
            Serial.println(F("Cruise control DISABLED (RC) — servo neutral"));
        }
    }

    uint32_t now = millis();
    uint32_t elapsed = now - prevTimeMs;

    if (elapsed < LOOP_MS) return;  // wait for next control tick

    uint16_t currentAngle = 0;
    if (!readAngle(currentAngle)) {
        Serial.println(F("ERROR: I2C read failed"));
        return;
    }

    // Compute speed
    measuredRPM = computeRPM(currentAngle, prevAngle, (float)elapsed);
    prevAngle   = currentAngle;
    prevTimeMs  = now;

    // Proportional controller (only when enabled)

    int pulseUs = SERVO_NEUTRAL;
    if (controlEnabled) {
        float error         = targetRPM - measuredRPM;
        float controlOutput = Kp * error;
        pulseUs = outputToServoPulse(controlOutput);
    }

    // Low-pass filter the servo output
    filteredPulseUs = FILTER_ALPHA * pulseUs + (1.0f - FILTER_ALPHA) * filteredPulseUs;

    // Slew rate limiter — clamp how fast the output can change
    {
        float maxDelta = SLEW_RATE_MAX_US_PER_MS * (float)elapsed;
        float delta    = filteredPulseUs - slewedPulseUs;
        if (delta >  maxDelta) delta =  maxDelta;
        if (delta < -maxDelta) delta = -maxDelta;
        slewedPulseUs += delta;
    }

    // Fast-drop override: snap to neutral immediately and reset all filter/slew
    // state so there is no ramp delay when the RC signal is low.
    if (rcPulseUs < RC_FAST_DROP_US) {
        filteredPulseUs = (float)SERVO_NEUTRAL;
        slewedPulseUs   = (float)SERVO_NEUTRAL;
    }

    int filteredPulseInt = (int)(slewedPulseUs + 0.5f);
    throttleServo.writeMicroseconds(filteredPulseInt);

    // Serial telemetry (CSV, compatible with Serial Plotter)
    Serial.print(controlEnabled ? 1 : 0);
    Serial.print(',');
    Serial.print(targetRPM, 1);
    Serial.print(',');
    Serial.print(measuredRPM, 1);
    Serial.print(',');
    Serial.print(filteredPulseInt);
    Serial.print(',');
    Serial.println(rcPulseUs);

    // Handle Serial commands:
    //   s<value>  → set target RPM   (e.g. "s120.5")
    //   k<value>  → set Kp gain      (e.g. "k3.0")
    while (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 's' || cmd == 'S') {
            targetRPM = Serial.parseFloat();
            Serial.print(F("Target RPM set to: "));
            Serial.println(targetRPM, 1);
        } else if (cmd == 'k' || cmd == 'K') {
            Kp = Serial.parseFloat();
            Serial.print(F("Kp set to: "));
            Serial.println(Kp, 3);
        }
    }
}
