#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>  // For trigonometric calculations

// Create MPU6050 objects for both sensors
Adafruit_MPU6050 mpu1;  // Sensor 1 (default I2C address 0x68)
Adafruit_MPU6050 mpu2;  // Sensor 2 (I2C address 0x69)

// Kalman filter state
float pitch1 = 0, pitch2 = 0;
float pitchOffset1 = 0, pitchOffset2 = 0;
const float correctionFactor = 0.93;

// Define vehicle parameters
const float Weight = 390;  // Vehicle weight in kg
const float height = 1.325;  // Height of center of gravity in meters
const float Length = 5.58;  // Wheelbase length in meters
float TENG = 142;          // Example engine torque in kg m
float l1 = 2.975;         // Distance from Center of Gravity to Front Axle
float r = 0.1;           // Rolling Resistance

// Define switch pins
const int switchPins[5] = {2, 3, 4, 5, 6};  // 5-position switch for mu values
const float positionValues[5] = {0.1, 0.3, 0.5, 0.7, 0.9};  // mu values

const int modePins[3] = {7, 8, 9};  // 3-position switch for drive modes
const int yellowLED = 10;
const int greenLED = 11;

unsigned long prevTime = 0;
unsigned long lastPrintTime = 0;
const unsigned long printInterval = 5000;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    Wire.begin();

    // Initialize MPU6050 Sensors
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

    // Initialize switch pins
    for (int i = 0; i < 5; i++) pinMode(switchPins[i], INPUT_PULLUP);
    for (int i = 0; i < 3; i++) pinMode(modePins[i], INPUT_PULLUP);
    pinMode(yellowLED, OUTPUT);
    pinMode(greenLED, OUTPUT);
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
    
    // Determine mu from 5-position switch
    float mu = 0;
    for (int i = 0; i < 5; i++) {
        if (digitalRead(switchPins[i]) == LOW) {
            mu = positionValues[i];
            break;
        }
    }

    // Determine drive mode from 3-position switch
    float torqueFront = 0;
    if (digitalRead(modePins[0]) == LOW) {  // 2WD
        torqueFront = 0;
        Serial.println("Mode: 2WD");
    } 
    else if (digitalRead(modePins[1]) == LOW) {  // 4WD
        if (mu != 0) {  // Prevent division by zero
            torqueFront = TENG - ((mu * Weight * cos(averagePitch*(PI/180))*((l1-(r*height))/Length)) / 
                          (1 - ((mu * height) / Length)));
        } else {
            torqueFront = 0;
        }
        Serial.println("Mode: 4WD");
    } 
    else if (digitalRead(modePins[2]) == LOW) {  // 4WD Lock
        torqueFront = TENG / 2;
        Serial.println("Mode: 4WD Lock");
    } 
    else {
        Serial.println("No mode selected");
    }

    // Print torque and mu values
    Serial.print("mu: ");
    Serial.print(mu, 2);
    Serial.print(" | Average Pitch: ");
    Serial.print(averagePitch, 2);
    Serial.print("° | Torque to Front: ");
    Serial.println(torqueFront, 2);

    // LED Indicator
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
    
   delay(500);  // Serial print frequency to once every 2 seconds

}

// Function to calibrate pitch
void calibratePitch(Adafruit_MPU6050& mpu, int sensor) {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);
    float pitch = atan2(accel.acceleration.y, accel.acceleration.z) * 180 / PI;
    if (sensor == 1) pitchOffset1 = pitch;
    else pitchOffset2 = pitch;
}

// Simple Kalman filter function
void kalmanFilter(float accelPitch, float gyroRate, float dt, int sensor) {
    if (sensor == 1) {
        pitch1 = pitch1 + gyroRate * dt + (accelPitch - pitch1) * correctionFactor;
    } else {
        pitch2 = pitch2 + gyroRate * dt + (accelPitch - pitch2) * correctionFactor;
    }
}
