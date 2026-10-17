##     FollowTheHand

An ARDUINO sketch using two HC-SR04 and two HY-SRF05 ultrasonic sensors, it triangulates the object position of the object nearest to the sensors, and animates a Unity 3d virtual head to look at that object.

This is a picture of the setup:

![Picture](/FollowTheHand/FollowTheHand.JPG)

The two HY-SRF05 ultrasonic sensors are located on top of the monitor, left and right of the middle of the monitor, they are called TL (top left) and TR (top right) in the code.
The two HY-SR04 ultrasonic sensors are located below the monitor, left and right of the middle of the monitor, they are called BL (bottom left) and BR (bottom right) in the code.

The position of the nearest object calculated from the sensor outputs is sent via Serial to a Unity 3d project. This project shows a very simple head with eyes, the head and eyes are animated so that they look at the nearest object. This is what it looks like:

https://github.com/user-attachments/assets/d95815f4-1e74-4599-b4b7-c438d0c05fbc


### Parts:

- ARDUINO UNO R4, and breadboard
- Two HC-SR04 ultrasonic sensors, the ones below the monitor
- Two HY-SRF05 ultrasonic sensors, the ones on to of the monitor
- Second breadboard for holding the sensor on the left
- Cables

### Software

- Standard ARDUINO software.

### Wiring:
The sketch uses digital pin D8 in order to trigger all 4 ultrasound sensors. The digital pins D3, D4, D5, and D6 are used for receiving the echos of the four sensors.
![Picture](/FollowTheHand/ArduinoWiring.JPG)

The sketch uses digital pin D8 in order to trigger all 4 ultrasound sensors simultaneously. 
![Picture](/FollowTheHand/TriggerPinWiring.JPG)

Wiring of the HY-SRF05 sensors:
![Picture](/FollowTheHand/HY-SRF06-Wiring.JPG)
From left to right, ground, echo, trigger, 5V.

Wiring of the HY-SR04 sensors:
![Picture](/FollowTheHand/HC-SR04-Wiring.JPG)
From left to right, 5V, trigger, echo, ground,.

### Code:
```text
/*
 FollowTheHand.ino - ARDUINO sketch for 4 ultrasound sensors.

MIT License

Copyright (C) 2026, Tamiko Thiel and Peter Graf

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

For more information on

Tamiko Thiel, see www.TamikoThiel.com/
Peter Graf, see www.mission-base.com/peter/

*/
#include <Arduino.h>
#include <math.h>

// ========================================================
// SIMULTANEOUS FOUR-SENSOR 3D HAND TRACKER
// Shared TRIG: D8 (all four sensors)
// ECHO: TL=D3, TR=D4, BL=D5, BR=D6
// Arduino UNO R4 WiFi
//
// Sensor positions:
//
// TL = ( 0, 52, -5)
// TR = (28, 52, -5)
// BL = ( 0,  0,  0)
// BR = (28,  0,  0)
//
// +X = your right
// +Y = upward
// +Z = toward you
//
// All four sensors fire simultaneously.
//
// Position algorithm:
//
// 1. <20 cm corner override.
// 2. Reject <5 cm and >70 cm.
// 3. If four readings valid, discard largest.
// 4. Trilaterate from remaining three.
// 5. If exactly three are valid, use those three.
// 6. Select solution in front of monitor.
//
// Unity output:
//
// POS,x,y,z
// ========================================================

const float MIN_VALID_DISTANCE = 5.0f;
const float MAX_VALID_DISTANCE = 70.0f;
const float CORNER_OVERRIDE_DISTANCE = 20.0f;
const float SINGLE_SENSOR_MAX_DISTANCE = 30.0f;

const int SHARED_TRIG_PIN = 8;
const int TL_ECHO_PIN = 3;
const int TR_ECHO_PIN = 4;
const int BL_ECHO_PIN = 5;
const int BR_ECHO_PIN = 6;

const unsigned long ECHO_TIMEOUT_US = 12000UL;
const unsigned long MEASUREMENT_INTERVAL_MS = 100;
const unsigned long DIAGNOSTIC_INTERVAL_MS = 500;
unsigned long lastDiagnosticMillis = 0;
const float TWO_SENSOR_TIME_CONSTANT_S = 1.0f;
const float RECOVERY_TIME_CONSTANT_S = 0.2f;
const float SENSOR_FILTER_ALPHA = 0.5f;
const byte FILTER_RESET_AFTER_INVALID = 3;

// ========================================================
// 3D VECTOR
// ========================================================

struct Vec3
{
    float x;
    float y;
    float z;
};

// ========================================================
// SENSOR
// ========================================================

struct Sensor
{
    const char *name;
    Vec3 position;
    float distance;
    bool valid;
};

Sensor sensors[4] =
{
    {"TL", {0.0f, 52.0f, -5.0f}, -1.0f, false},
    {"TR", {28.0f, 52.0f, -5.0f}, -1.0f, false},
    {"BL", {0.0f, 0.0f, 0.0f}, -1.0f, false},
    {"BR", {28.0f, 0.0f, 0.0f}, -1.0f, false}
};

// ========================================================
// SENSOR LOW-PASS FILTERS
// ========================================================

struct DistanceFilter
{
    float value;
    bool initialized;
    byte invalidCount;
};

DistanceFilter distanceFilters[4] = {};
float rawDistances[4] = {-1.0f, -1.0f, -1.0f, -1.0f};

float filterDistance(int index, float raw)
{
    DistanceFilter &filter = distanceFilters[index];
    if (raw < MIN_VALID_DISTANCE || raw > MAX_VALID_DISTANCE)
    {
        if (filter.invalidCount < FILTER_RESET_AFTER_INVALID)
        {
            filter.invalidCount++;
        }
        if (filter.invalidCount >= FILTER_RESET_AFTER_INVALID)
        {
            filter.initialized = false;
        }
        return -1.0f;
    }

    filter.invalidCount = 0;
    if (!filter.initialized)
    {
        filter.value = raw;
        filter.initialized = true;
    }
    else
    {
        filter.value += SENSOR_FILTER_ALPHA * (raw - filter.value);
    }
    return filter.value;
}

// ========================================================
// ECHO STATE
// ========================================================

struct EchoMeasurement
{
    int pin;
    bool started;
    bool finished;
    unsigned long startTime;
    unsigned long endTime;
};

EchoMeasurement echoTL;
EchoMeasurement echoTR;
EchoMeasurement echoBL;
EchoMeasurement echoBR;

// Explicit prototypes prevent Arduino auto-generation before custom struct definitions.
Vec3 subtract(Vec3 a, Vec3 b);
Vec3 add(Vec3 a, Vec3 b);
Vec3 multiply(Vec3 a, float value);
float dot(Vec3 a, Vec3 b);
Vec3 cross(Vec3 a, Vec3 b);
float lengthVec(Vec3 a);
Vec3 normalize(Vec3 a);
float distanceVec(Vec3 a, Vec3 b);
void resetEcho(EchoMeasurement &echo, int pin);
void updateEcho(EchoMeasurement &echo);
float echoToDistance(const EchoMeasurement &echo);
void triggerAllSensors();
bool trilaterate3(const Sensor &s1, const Sensor &s2, const Sensor &s3, Vec3 &result);
bool neighboringSensors(int a, int b);
bool estimateTwoSensors(int a, int b, float dt, Vec3 &result);
Vec3 smoothRecovery(Vec3 calculated, float dt);
void rememberPosition(Vec3 position);
void sendPositionToUnity(Vec3 position);
void printSensor(const Sensor &sensor);
void setup();
void loop();

// ========================================================
// VECTOR FUNCTIONS
// ========================================================

Vec3 subtract(Vec3 a, Vec3 b)
{
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 add(Vec3 a, Vec3 b)
{
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 multiply(Vec3 a, float value)
{
    return {a.x * value, a.y * value, a.z * value};
}

float dot(Vec3 a, Vec3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 cross(Vec3 a, Vec3 b)
{
    return
    {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

float lengthVec(Vec3 a)
{
    return sqrt(dot(a, a));
}

Vec3 normalize(Vec3 a)
{
    float length = lengthVec(a);

    if (length <= 0.0001f)
    {
        return {0.0f, 0.0f, 0.0f};
    }

    return multiply(a, 1.0f / length);
}

float distanceVec(Vec3 a, Vec3 b)
{
    return lengthVec(subtract(a, b));
}

// ========================================================
// ECHO FUNCTIONS
// ========================================================

void resetEcho(EchoMeasurement &echo, int pin)
{
    echo.pin = pin;
    echo.started = false;
    echo.finished = false;
    echo.startTime = 0;
    echo.endTime = 0;
}

void updateEcho(EchoMeasurement &echo)
{
    int state = digitalRead(echo.pin);

    if (!echo.started && state == HIGH)
    {
        echo.started = true;
        echo.startTime = micros();
    }

    if (echo.started && !echo.finished && state == LOW)
    {
        echo.finished = true;
        echo.endTime = micros();
    }
}

float echoToDistance(const EchoMeasurement &echo)
{
    if (!echo.started || !echo.finished)
    {
        return -1.0f;
    }

    unsigned long duration = echo.endTime - echo.startTime;
    float distance = duration * 0.0343f / 2.0f;

    if (distance < MIN_VALID_DISTANCE || distance > MAX_VALID_DISTANCE)
    {
        return -1.0f;
    }

    return distance;
}

// ========================================================
// TRIGGER ALL FOUR
// ========================================================

void triggerAllSensors()
{
    digitalWrite(SHARED_TRIG_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(SHARED_TRIG_PIN, HIGH);
    delayMicroseconds(15);
    digitalWrite(SHARED_TRIG_PIN, LOW);
}

// ========================================================
// THREE-SPHERE TRILATERATION
// ========================================================
//
// Standard trilateration:
//
// P1, P2, P3 = sensor positions
// r1, r2, r3 = measured distances
//
// Produces two possible positions.
// We choose the solution with the larger positive Z.
// ========================================================

bool trilaterate3(const Sensor &s1, const Sensor &s2, const Sensor &s3, Vec3 &result)
{
    Vec3 p1 = s1.position;
    Vec3 p2 = s2.position;
    Vec3 p3 = s3.position;
    float r1 = s1.distance;
    float r2 = s2.distance;
    float r3 = s3.distance;
    Vec3 p2p1 = subtract(p2, p1);
    float d = lengthVec(p2p1);

    if (d < 0.001f)
    {
        return false;
    }

    Vec3 ex = normalize(p2p1);
    Vec3 p3p1 = subtract(p3, p1);
    float i = dot(ex, p3p1);
    Vec3 temp = subtract(p3p1, multiply(ex, i));
    float tempLength = lengthVec(temp);

    if (tempLength < 0.001f)
    {
        return false;
    }

    Vec3 ey = normalize(temp);
    Vec3 ez = cross(ex, ey);
    float j = dot(ey, p3p1);

    if (fabs(j) < 0.001f)
    {
        return false;
    }

    float x = (r1 * r1 - r2 * r2 + d * d) / (2.0f * d);
    float y = (r1 * r1 - r3 * r3 + i * i + j * j - 2.0f * i * x) / (2.0f * j);
    float zSquared = r1 * r1 - x * x - y * y;

    if (zSquared < 0.0f)
    {
        return false;
    }

    float z = sqrt(zSquared);
    Vec3 base = add(p1, add(multiply(ex, x), multiply(ey, y)));
    Vec3 solution1 = add(base, multiply(ez, z));
    Vec3 solution2 = add(base, multiply(ez, -z));

    if (solution1.z >= 0.0f && solution2.z < 0.0f)
    {
        result = solution1;
        return true;
    }

    if (solution2.z >= 0.0f && solution1.z < 0.0f)
    {
        result = solution2;
        return true;
    }

    if (solution1.z >= 0.0f && solution2.z >= 0.0f)
    {
        if (solution1.z >= solution2.z)
        {
            result = solution1;
        }
        else
        {
            result = solution2;
        }

        return true;
    }

    return false;
}

// ========================================================
// TRACKING STATE AND TWO-SENSOR FALLBACK
// ========================================================

Vec3 lastPosition = {14.0f, 19.5f, 50.0f};
bool hasLastPosition = false;
bool inTwoSensorMode = false;
unsigned long lastOutputMillis = 0;

bool neighboringSensors(int a, int b)
{
    return (a == 0 && b == 1) || (a == 2 && b == 3) ||
           (a == 0 && b == 2) || (a == 1 && b == 3);
}

// Two ranges determine a circle in 3D. The previous position
// selects a point on that circle; its unconstrained coordinate
// gradually approaches the plane containing the sensor pair.
bool estimateTwoSensors(int a, int b, float dt, Vec3 &result)
{
    if (!neighboringSensors(a, b))
    {
        return false;
    }
    Vec3 p1 = sensors[a].position;
    Vec3 p2 = sensors[b].position;
    float r1 = sensors[a].distance;
    float r2 = sensors[b].distance;
    Vec3 baseline = subtract(p2, p1);
    float d = lengthVec(baseline);
    Vec3 axis = multiply(baseline, 1.0f / d);
    float along = (r1 * r1 - r2 * r2 + d * d) / (2.0f * d);
    float radius2 = r1 * r1 - along * along;
    if (radius2 < 0.0f)
    {
        return false;
    }
    Vec3 base = add(p1, multiply(axis, along));
    float alpha = 1.0f - expf(-dt / TWO_SENSOR_TIME_CONSTANT_S);
    Vec3 previous = hasLastPosition ? lastPosition : base;
    bool horizontal = fabsf(p1.y - p2.y) < 0.001f;
    if (horizontal)
    {
        float targetY = p1.y;
        float previousY = hasLastPosition ? previous.y : targetY;
        float y = previousY + alpha * (targetY - previousY);
        float offsetY = y - targetY;
        float maxOffset = sqrtf(radius2);
        offsetY = constrain(offsetY, -maxOffset, maxOffset);
        y = targetY + offsetY;
        float z = p1.z + sqrtf(fmaxf(0.0f, radius2 - offsetY * offsetY));
        if (z < 0.0f)
        {
            return false;
        }
        result = {base.x, y, z};
        return true;
    }
    float targetX = p1.x;
    float previousX = hasLastPosition ? previous.x : targetX;
    float x = previousX + alpha * (targetX - previousX);
    float offsetX = x - targetX;
    float maxOffset = sqrtf(radius2);
    offsetX = constrain(offsetX, -maxOffset, maxOffset);
    x = targetX + offsetX;
    float radius = sqrtf(fmaxf(0.0f, radius2 - offsetX * offsetX));
    Vec3 perpendicular = {0.0f, -axis.z, axis.y};
    Vec3 candidate1 = add(base, multiply(perpendicular, radius));
    Vec3 candidate2 = add(base, multiply(perpendicular, -radius));
    Vec3 chosen = candidate1.z >= candidate2.z ? candidate1 : candidate2;
    if (chosen.z < 0.0f)
    {
        return false;
    }
    result = {x, chosen.y, chosen.z};
    return true;
}

Vec3 smoothRecovery(Vec3 calculated, float dt)
{
    if (!inTwoSensorMode || !hasLastPosition)
    {
        return calculated;
    }
    float alpha = 1.0f - expf(-dt / RECOVERY_TIME_CONSTANT_S);
    return add(lastPosition, multiply(subtract(calculated, lastPosition), alpha));
}

void rememberPosition(Vec3 position)
{
    lastPosition = position;
    hasLastPosition = true;
    lastOutputMillis = millis();
}

// ========================================================
// UNITY OUTPUT
// ========================================================

void sendPositionToUnity(Vec3 position)
{
    Serial.print("POS,");
    Serial.print(position.x, 1);
    Serial.print(",");
    Serial.print(position.y, 1);
    Serial.print(",");
    Serial.println(position.z, 1);
}

// ========================================================
// RAW OUTPUT
// ========================================================

void printSensor(const Sensor &sensor)
{
    Serial.print(sensor.name);
    Serial.print("=");

    if (!sensor.valid)
    {
        Serial.print("---");
    }
    else
    {
        Serial.print(sensor.distance, 1);
    }
}

// ========================================================
// SETUP
// ========================================================

void setup()
{
    Serial.begin(115200);
    pinMode(SHARED_TRIG_PIN, OUTPUT);
    pinMode(TL_ECHO_PIN, INPUT);
    pinMode(TR_ECHO_PIN, INPUT);
    pinMode(BL_ECHO_PIN, INPUT);
    pinMode(BR_ECHO_PIN, INPUT);
    digitalWrite(SHARED_TRIG_PIN, LOW);
    delay(1000);
    Serial.println();
    Serial.println("========================================");
    Serial.println(" THREE-OF-FOUR 3D HAND TRACKER");
    Serial.println(" FILTERED DISTANCES, TWO-SENSOR FALLBACK");
    Serial.println("========================================");
    Serial.println();
}

// ========================================================
// LOOP
// ========================================================

void loop()
{
    unsigned long now = millis();
    bool printDiagnostics = (now - lastDiagnosticMillis >= DIAGNOSTIC_INTERVAL_MS);
    if (printDiagnostics)
    {
        lastDiagnosticMillis = now;
    }
    resetEcho(echoTL, TL_ECHO_PIN);
    resetEcho(echoTR, TR_ECHO_PIN);
    resetEcho(echoBL, BL_ECHO_PIN);
    resetEcho(echoBR, BR_ECHO_PIN);
    triggerAllSensors();
    unsigned long measurementStart = micros();

    while (micros() - measurementStart < ECHO_TIMEOUT_US)
    {
        updateEcho(echoTL);
        updateEcho(echoTR);
        updateEcho(echoBL);
        updateEcho(echoBR);

        if (echoTL.finished && echoTR.finished && echoBL.finished && echoBR.finished)
        {
            break;
        }
    }

    rawDistances[0] = echoToDistance(echoTL);
    rawDistances[1] = echoToDistance(echoTR);
    rawDistances[2] = echoToDistance(echoBL);
    rawDistances[3] = echoToDistance(echoBR);

    for (int i = 0; i < 4; i++)
    {
        sensors[i].distance = filterDistance(i, rawDistances[i]);
        sensors[i].valid = sensors[i].distance >= MIN_VALID_DISTANCE && sensors[i].distance <= MAX_VALID_DISTANCE;
    }

    if (printDiagnostics)
    {
        printSensor(sensors[0]);
        Serial.print("  ");
        printSensor(sensors[1]);
        Serial.print("  ");
        printSensor(sensors[2]);
        Serial.print("  ");
        printSensor(sensors[3]);
    }

    // ====================================================
    // CORNER OVERRIDE
    // ====================================================

    int cornerSensor = -1;
    float cornerDistance = 10000.0f;

    for (int i = 0; i < 4; i++)
    {
        if (rawDistances[i] >= MIN_VALID_DISTANCE && rawDistances[i] < CORNER_OVERRIDE_DISTANCE && rawDistances[i] < cornerDistance)
        {
            cornerSensor = i;
            cornerDistance = rawDistances[i];
        }
    }

    if (cornerSensor >= 0)
    {
        Vec3 position = sensors[cornerSensor].position;
        position.z += cornerDistance;
        if (printDiagnostics)
        {
            Serial.print("  CORNER=");
            Serial.print(sensors[cornerSensor].name);
            Serial.print("  X=");
            Serial.print(position.x, 1);
            Serial.print(" Y=");
            Serial.print(position.y, 1);
            Serial.print(" Z=");
            Serial.println(position.z, 1);
        }
        sendPositionToUnity(position);
        rememberPosition(position);
        inTwoSensorMode = false;
        delay(MEASUREMENT_INTERVAL_MS);
        return;
    }

    // ====================================================
    // SELECT THREE SENSORS
    // ====================================================

    int validCount = 0;
    int validIndices[4];

    for (int i = 0; i < 4; i++)
    {
        if (sensors[i].valid)
        {
            validIndices[validCount] = i;
            validCount++;
        }
    }

    // One valid sensor: estimate directly in front only within 30 cm.
    // The existing raw <20 cm corner override above always takes priority.
    if (validCount == 1)
    {
        int index = validIndices[0];
        if (sensors[index].distance <= SINGLE_SENSOR_MAX_DISTANCE)
        {
            Vec3 position = sensors[index].position;
            position.z += sensors[index].distance;
            if (printDiagnostics)
            {
                Serial.print("  SINGLE SENSOR=");
                Serial.print(sensors[index].name);
                Serial.print(" X=");
                Serial.print(position.x, 1);
                Serial.print(" Y=");
                Serial.print(position.y, 1);
                Serial.print(" Z=");
                Serial.println(position.z, 1);
            }
            sendPositionToUnity(position);
            rememberPosition(position);
            inTwoSensorMode = false;
        }
        else
        {
            if (printDiagnostics)
            {
                Serial.println("  SINGLE SENSOR: HOLD LAST POSITION");
            }
        }
        delay(MEASUREMENT_INTERVAL_MS);
        return;
    }

    // Two neighboring sensors: continue tracking using last position.
    if (validCount == 2)
    {
        int a = validIndices[0];
        int b = validIndices[1];
        float dt = hasLastPosition ? (millis() - lastOutputMillis) / 1000.0f : MEASUREMENT_INTERVAL_MS / 1000.0f;
        Vec3 estimate;
        if (estimateTwoSensors(a, b, dt, estimate))
        {
            if (printDiagnostics)
            {
                Serial.print("  TWO SENSOR=");
                Serial.print(sensors[a].name);
                Serial.print("-");
                Serial.print(sensors[b].name);
                Serial.print(" X=");
                Serial.print(estimate.x, 1);
                Serial.print(" Y=");
                Serial.print(estimate.y, 1);
                Serial.print(" Z=");
                Serial.println(estimate.z, 1);
            }
            sendPositionToUnity(estimate);
            rememberPosition(estimate);
            inTwoSensorMode = true;
        }
        else
        {
            if (printDiagnostics)
            {
                Serial.println("  TWO SENSOR: NO VALID ESTIMATE");
            }
        }
        delay(MEASUREMENT_INTERVAL_MS);
        return;
    }

    if (validCount < 3)
    {
        if (printDiagnostics)
        {
            Serial.println("  NOT ENOUGH SENSORS");
        }
        delay(MEASUREMENT_INTERVAL_MS);
        return;
    }

    int excludedSensor = -1;

    if (validCount == 4)
    {
        int largestPosition = 0;

        for (int i = 1; i < 4; i++)
        {
            if (sensors[validIndices[i]].distance > sensors[validIndices[largestPosition]].distance)
            {
                largestPosition = i;
            }
        }

        excludedSensor = validIndices[largestPosition];
        int selected[3];
        int selectedCount = 0;

        for (int i = 0; i < 4; i++)
        {
            if (validIndices[i] != excludedSensor)
            {
                selected[selectedCount] = validIndices[i];
                selectedCount++;
            }
        }

        validIndices[0] = selected[0];
        validIndices[1] = selected[1];
        validIndices[2] = selected[2];
    }

    // ====================================================
    // TRILATERATE
    // ====================================================

    Vec3 position;
    bool solved = trilaterate3(sensors[validIndices[0]], sensors[validIndices[1]], sensors[validIndices[2]], position);

    if (!solved)
    {
        if (printDiagnostics)
        {
            Serial.println("  NO 3D SOLUTION");
        }
        delay(MEASUREMENT_INTERVAL_MS);
        return;
    }

    // Smooth the first recovered 3D position after two-sensor mode.
    if (inTwoSensorMode && hasLastPosition)
    {
        float dt = (millis() - lastOutputMillis) / 1000.0f;
        position = smoothRecovery(position, dt);
        inTwoSensorMode = false;
    }

    // ====================================================
    // OUTPUT
    // ====================================================

    if (printDiagnostics)
    {
        if (excludedSensor >= 0)
        {
            float predictedExcludedDistance = distanceVec(position, sensors[excludedSensor].position);
            float excludedError = fabs(predictedExcludedDistance - sensors[excludedSensor].distance);
            Serial.print("  IGNORE=");
            Serial.print(sensors[excludedSensor].name);
            Serial.print(" ERR=");
            Serial.print(excludedError, 1);
        }
        else
        {
            Serial.print("  THREE VALID");
        }
    
        Serial.print("  X=");
        Serial.print(position.x, 1);
        Serial.print(" Y=");
        Serial.print(position.y, 1);
        Serial.print(" Z=");
        Serial.println(position.z, 1);
    }
    sendPositionToUnity(position);
    rememberPosition(position);
    delay(MEASUREMENT_INTERVAL_MS);
}
```


