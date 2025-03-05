#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Create MPU6050 objects for both sensors
Adafruit_MPU6050 mpu1;  // Sensor 1 (default I2C address 0x68)
Adafruit_MPU6050 mpu2;  // Sensor 2 (I2C address 0x69)

// Kalman filter state
float pitch1 = 0;          // Filtered pitch value for sensor 1
float pitch2 = 0;          // Filtered pitch value for sensor 2
float bias1 = 0;           // Bias of the gyro for sensor 1
float bias2 = 0;           // Bias of the gyro for sensor 2
float P1[2][2] = {{1, 0}, {0, 1}};  // Error covariance matrix for sensor 1
float P2[2][2] = {{1, 0}, {0, 1}};  // Error covariance matrix for sensor 2
float Q_angle = 0.001;     // Process noise variance for accelerometer
float Q_bias = 0.003;      // Process noise variance for gyroscope bias
float R_measure = 0.03;    // Measurement noise variance

float pitchOffset1 = 0;    // Offset for sensor 1
float pitchOffset2 = 0;    // Offset for sensor 2

unsigned long prevTime = 0; // Previous time for delta time calculation

const float correctionFactor = 0.93;  // Scaling factor to fix the 90-degree issue

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10); 

  Wire.begin();

  // Initialize Sensor 1 (I2C address 0x68)
  if (!mpu1.begin(0x68)) {  // Default I2C address 0x68
    Serial.println("Failed to find MPU6050 sensor 1! Check your wiring.");
    while (1);
  }
  Serial.println("MPU6050 sensor 1 initialized!");

  // Initialize Sensor 2 (I2C address 0x69)
  if (!mpu2.begin(0x69)) {  // I2C address 0x69
    Serial.println("Failed to find MPU6050 sensor 2! Check your wiring.");
    while (1);
  }
  Serial.println("MPU6050 sensor 2 initialized!");

  // Configure both sensors
  mpu1.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu1.setGyroRange(MPU6050_RANGE_250_DEG);

  mpu2.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu2.setGyroRange(MPU6050_RANGE_250_DEG);

  // Calibrate both sensors
  calibratePitch(mpu1, 1);
  calibratePitch(mpu2, 2);

  prevTime = micros(); 
}

void loop() {
  sensors_event_t accel1, gyro1, temp1;
  sensors_event_t accel2, gyro2, temp2;

  // Read data from Sensor 1
  mpu1.getEvent(&accel1, &gyro1, &temp1);

  // Read data from Sensor 2
  mpu2.getEvent(&accel2, &gyro2, &temp2);

  // Calculate delta time (dt) in seconds using micros() for higher precision
  unsigned long currentTime = micros();
  float dt = (currentTime - prevTime) / 1000000.0; // Convert microseconds to seconds
  prevTime = currentTime;

  // Calculate pitch from accelerometer for Sensor 1
  float accelPitch1 = atan2(accel1.acceleration.y, accel1.acceleration.z) * 180 / PI;

  // Calculate pitch from accelerometer for Sensor 2
  float accelPitch2 = atan2(accel2.acceleration.y, accel2.acceleration.z) * 180 / PI;

  // Gyroscope rate (degrees/sec) for Sensor 1 and Sensor 2
  float gyroRate1 = gyro1.gyro.x * 180 / PI;
  float gyroRate2 = gyro2.gyro.x * 180 / PI;

  // Apply Kalman Filter for both sensors
  kalmanFilter(accelPitch1, gyroRate1, dt, 1);
  kalmanFilter(accelPitch2, gyroRate2, dt, 2);

  // Subtract calibration offset for both sensors
  float calibratedPitch1 = pitch1 - pitchOffset1;
  float calibratedPitch2 = pitch2 - pitchOffset2;

  // Apply the correction factor for tilt beyond 90 degrees for both sensors
  calibratedPitch1 *= correctionFactor;
  calibratedPitch2 *= correctionFactor;

  // Apply the adjustment to subtract 3 or add 3 if pitch is above or below 0 (except if close to 0)
  if (calibratedPitch1 > 2) {
    calibratedPitch1 -= 3.5;  // Subtract 3 if pitch is greater than 2
  } else if (calibratedPitch1 < -2) {
    calibratedPitch1 += 3.5;  // Add 3 if pitch is less than -2
  }

  if (calibratedPitch2 > 2) {
    calibratedPitch2 -= 7;  // Subtract 3 if pitch is greater than 2
  } else if (calibratedPitch2 < -2) {
    calibratedPitch2 += 7;  // Add 3 if pitch is less than -2
  }

  Serial.print("sensor 2: ");
  Serial.print(calibratedPitch2, 2);  // Print with 2 decimal places
  Serial.println(" degrees");

  // Calculate the average pitch value from both sensors
  //float averagePitch = (calibratedPitch1 + calibratedPitch2) / 2.0;

  // Print the average pitch value of both sensors
  // Serial.print("Average Pitch: ");
  // Serial.print(averagePitch, 2);  // Print with 2 decimal places
  // Serial.println(" degrees");
}

// Kalman Filter Implementation for both sensors
void kalmanFilter(float accelAngle, float gyroRate, float dt, int sensor) {
  // Step 1: Predict
  if (sensor == 1) {
    pitch1 += dt * (gyroRate - bias1);  // Update pitch estimate using gyroscope rate for sensor 1
    P1[0][0] += dt * (dt * P1[1][1] - P1[0][1] - P1[1][0] + Q_angle);
    P1[0][1] -= dt * P1[1][1];
    P1[1][0] -= dt * P1[1][1];
    P1[1][1] += Q_bias * dt;
  } else if (sensor == 2) {
    pitch2 += dt * (gyroRate - bias2);  // Update pitch estimate using gyroscope rate for sensor 2
    P2[0][0] += dt * (dt * P2[1][1] - P2[0][1] - P2[1][0] + Q_angle);
    P2[0][1] -= dt * P2[1][1];
    P2[1][0] -= dt * P2[1][1];
    P2[1][1] += Q_bias * dt;
  }

  // Step 2: Update
  float y = accelAngle - (sensor == 1 ? pitch1 : pitch2);  // Angle difference for respective sensor
  float S = (sensor == 1 ? P1[0][0] : P2[0][0]) + R_measure;  // Estimate error
  float K[2];  // Kalman gain
  K[0] = (sensor == 1 ? P1[0][0] : P2[0][0]) / S;
  K[1] = (sensor == 1 ? P1[1][0] : P2[1][0]) / S;

  // Adjust pitch and bias for respective sensor
  if (sensor == 1) {
    pitch1 += K[0] * y;
    bias1 += K[1] * y;
  } else if (sensor == 2) {
    pitch2 += K[0] * y;
    bias2 += K[1] * y;
  }

  // Update the error covariance matrix
  float P00_temp = (sensor == 1 ? P1[0][0] : P2[0][0]);
  float P01_temp = (sensor == 1 ? P1[0][1] : P2[0][1]);

  if (sensor == 1) {
    P1[0][0] -= K[0] * P00_temp;
    P1[0][1] -= K[0] * P01_temp;
    P1[1][0] -= K[1] * P00_temp;
    P1[1][1] -= K[1] * P01_temp;
  } else if (sensor == 2) {
    P2[0][0] -= K[0] * P00_temp;
    P2[0][1] -= K[0] * P01_temp;
    P2[1][0] -= K[1] * P00_temp;
    P2[1][1] -= K[1] * P01_temp;
  }
}

// Function to calibrate pitch offset for both sensors (only when sensor is level)
void calibratePitch(Adafruit_MPU6050 &mpu, int sensor) {
  sensors_event_t accel, temp;

  // Read accelerometer data
  mpu.getEvent(&accel, &temp, &temp);

  // Calculate the pitch angle from accelerometer data
  float accelPitch = atan2(accel.acceleration.y, accel.acceleration.z) * 180 / PI;

  // Check if the sensor is near level (within a smaller tolerance)
  if (fabs(accelPitch) < 2.0) {  // 2 degrees tolerance for being level
    // Set the pitch offset as the calibration value for respective sensor
    if (sensor == 1) {
      pitchOffset1 = accelPitch;  // Save the initial reading as offset
      Serial.print("Sensor 1 - Calibrated Pitch Offset: ");
      Serial.println(pitchOffset1, 2);
    } else if (sensor == 2) {
      pitchOffset2 = accelPitch;  // Save the initial reading as offset
      Serial.print("Sensor 2 - Calibrated Pitch Offset: ");
      Serial.println(pitchOffset2, 2);
    }
  }
}
