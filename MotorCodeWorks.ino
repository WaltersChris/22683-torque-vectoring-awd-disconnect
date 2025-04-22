#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <AccelStepper.h>

Adafruit_MPU6050 mpu1;
Adafruit_MPU6050 mpu2;

float pitch1 = 0, pitch2 = 0;
float pitchOffset1 = 0, pitchOffset2 = 0;
const float correctionFactor = 0.93;

const float Weight = 390;
const float height = 1.325;
const float Length = 5.58;
float TENG = 142;
float l1 = 2.975;
float r = 0.1;

const int switchPins[5] = {31, 32, 33, 5, 6};
const float positionValues[5] = {0.1, 0.3, 0.5, 0.7, 0.9};

const int modePins[3] = {7, 8, 9};
const int yellowLED = 10;
const int greenLED = 11;

#define STEP_PIN 2
#define DIR_PIN 3
#define ENA_PIN 4
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

const int numIntervals = 160;
float torqueRangeMin = 0;
float torqueRangeMax = 160;
float intervalSize = 1.0;
float degreeStep = 4.5;

int lastStepIndex = -1;

unsigned long prevTime = 0;
float lastMu = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    Wire.begin();

    if (!mpu1.begin(0x68) || !mpu2.begin(0x69)) {
        Serial.println("MPU6050 initialization failed! Check wiring.");
        while (1);
    }

    mpu1.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu1.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu2.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu2.setGyroRange(MPU6050_RANGE_250_DEG);

    calibratePitch(mpu1, 1);
    calibratePitch(mpu2, 2);

    prevTime = micros();

    for (int i = 0; i < 5; i++) pinMode(switchPins[i], INPUT_PULLUP);
    for (int i = 0; i < 3; i++) pinMode(modePins[i], INPUT_PULLUP);
    pinMode(yellowLED, OUTPUT);
    pinMode(greenLED, OUTPUT);

    pinMode(ENA_PIN, OUTPUT);
    digitalWrite(ENA_PIN, LOW);

    stepper.setMaxSpeed(5000);
    stepper.setAcceleration(2000);
    stepper.setCurrentPosition(0);
}

void loop() {
    sensors_event_t accel1, gyro1, temp1, accel2, gyro2, temp2;
    mpu1.getEvent(&accel1, &gyro1, &temp1);
    mpu2.getEvent(&accel2, &gyro2, &temp2);

    unsigned long currentTime = micros();
    float dt = (currentTime - prevTime) / 1000000.0;
    prevTime = currentTime;

    float accelPitch1 = atan2(accel1.acceleration.y, accel1.acceleration.z) * 180 / PI;
    float accelPitch2 = atan2(accel2.acceleration.y, accel2.acceleration.z) * 180 / PI;

    float gyroRate1 = gyro1.gyro.x * 180 / PI;
    float gyroRate2 = gyro2.gyro.x * 180 / PI;

    kalmanFilter(accelPitch1, gyroRate1, dt, 1);
    kalmanFilter(accelPitch2, gyroRate2, dt, 2);

    float calibratedPitch1 = (pitch1 - pitchOffset1) * correctionFactor;
    float calibratedPitch2 = (pitch2 - pitchOffset2) * correctionFactor;

    if (calibratedPitch1 > 2) calibratedPitch1 -= 3.5;
    else if (calibratedPitch1 < -2) calibratedPitch1 += 3.5;

    if (calibratedPitch2 > 2) calibratedPitch2 -= 7;
    else if (calibratedPitch2 < -2) calibratedPitch2 += 7;

    float averagePitch = (calibratedPitch1 + calibratedPitch2) / 2.0;

    float mu = 0;
    bool switchDetected = false;
    for (int i = 0; i < 5; i++) {
        if (digitalRead(switchPins[i]) == LOW) {
            mu = positionValues[i];
            lastMu = mu;
            switchDetected = true;
            break;
        }
    }
    if (!switchDetected) {
        mu = lastMu;
    }

    float torqueFront = 0;
    if (digitalRead(modePins[0]) == LOW) {
        torqueFront = 0;
        Serial.println("Mode: 2WD");
    } 
    else if (digitalRead(modePins[1]) == LOW) {
        if (mu != 0) {
            torqueFront = TENG - ((mu * Weight * cos(averagePitch * (PI / 180)) * ((l1 - (r * height)) / Length)) /
                          (1 - ((mu * height) / Length)));
        } else {
            torqueFront = 0;
        }
        Serial.println("Mode: 4WD");
    } 
    else if (digitalRead(modePins[2]) == LOW) {
        torqueFront = TENG / 2;
        Serial.println("Mode: 4WD Lock");
    } 
    else {
        Serial.println("No mode selected");
    }

    Serial.print("mu: ");
    Serial.print(mu, 2);
    Serial.print(" | Avg Pitch: ");
    Serial.print(averagePitch, 2);
    Serial.print("° | Torque Front: ");
    Serial.println(torqueFront, 2);

    if (digitalRead(modePins[0]) == LOW) {
        digitalWrite(yellowLED, HIGH);
        digitalWrite(greenLED, LOW);
    } 
    else if (digitalRead(modePins[1]) == LOW || digitalRead(modePins[2]) == LOW) {
        digitalWrite(yellowLED, LOW);
        digitalWrite(greenLED, HIGH);
    } 
    else {
        digitalWrite(yellowLED, LOW);
        digitalWrite(greenLED, LOW);
    }


    if (mu > 0) {
    int intervalIndex = constrain((int)(torqueFront), 0, numIntervals - 1); // 1Nm steps

    if (intervalIndex != lastStepIndex) {
        int targetDegree = intervalIndex * degreeStep; // 4.5 degrees per Nm
        int targetSteps = map(targetDegree, 0, 360, 0, 1000); // Assuming 1000 steps = 360 degrees
        stepper.moveTo(targetSteps);
        stepper.runToPosition();
        lastStepIndex = intervalIndex;
    }
} else {
    Serial.println("mu is 0 — stepper motor is on hold.");
}


    delay(5);
}

void calibratePitch(Adafruit_MPU6050& mpu, int sensor) {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);
    float pitch = atan2(accel.acceleration.y, accel.acceleration.z) * 180 / PI;
    if (sensor == 1) pitchOffset1 = pitch;
    else pitchOffset2 = pitch;
}

void kalmanFilter(float accelPitch, float gyroRate, float dt, int sensor) {
    if (sensor == 1) {
        pitch1 = pitch1 + gyroRate * dt + (accelPitch - pitch1) * correctionFactor;
    } else {
        pitch2 = pitch2 + gyroRate * dt + (accelPitch - pitch2) * correctionFactor;
    }
}
