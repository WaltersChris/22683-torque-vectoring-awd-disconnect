#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Create MPU6050 object for the sensor
Adafruit_MPU6050 mpu;

// Variables for Kalman Filter
float pitch = 0;         // Filtered pitch value
float bias = 0;          // Bias of the gyro
float P[2][2] = {{1, 0}, {0, 1}}; // Error covariance matrix
float Q_angle = 0.001;   // Process noise variance for the accelerometer
float Q_bias = 0.003;    // Process noise variance for the gyroscope bias
float R_measure = 0.03;  // Measurement noise variance

float pitchOffset = 0;   // Offset to calibrate the flat ground position

unsigned long prevTime = 0; // Previous time for delta time calculation

// Fine-tuning constant to adjust small bias (this can now be set to 0 for simplicity)
const float fineTuneBias = 0.0;  // Remove the adjustment since we want 0 on flat ground
const float correctionFactor = 0.93;  // Scaling factor to fix the 90-degree issue 

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10); 

  // Initialize I2C communication
  Wire.begin();

  // Initialize the MPU6050 sensor
  if (!mpu.begin(0x68)) {  // Default I2C address is 0x68
    Serial.println("Failed to find MPU6050! Check your wiring.");
    while (1);
  }
  Serial.println("MPU6050 initialized!");

  // Configure the sensor
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);

  // Calibration step to determine pitch offset (only when sensor is level)
  calibratePitch();

  prevTime = micros(); 
}

void loop() {
  sensors_event_t accel, gyro, temp;

  // Read Sensor Data
  mpu.getEvent(&accel, &gyro, &temp);

  // Calculate delta time (dt) in seconds using micros() for higher precision
  unsigned long currentTime = micros();
  float dt = (currentTime - prevTime) / 1000000.0; // Convert microseconds to seconds
  prevTime = currentTime;

  // Calculate pitch from accelerometer
  float accelPitch = atan2(accel.acceleration.y, accel.acceleration.z) * 180 / PI;

  // Gyroscope rate (degrees/sec)
  float gyroRate = gyro.gyro.x * 180 / PI;

  // Apply Kalman Filter
  kalmanFilter(accelPitch, gyroRate, dt);

  // Subtract calibration offset
  float calibratedPitch = pitch - pitchOffset;

  // Apply the correction factor for tilt beyond 90 degrees
  calibratedPitch *= correctionFactor;

  // Additional offset correction for the sensor is reading -0.4 degrees on flat ground
  calibratedPitch += 0.4;  

  // Print filtered and calibrated pitch
  Serial.print("Filtered Pitch: ");
  Serial.println(calibratedPitch);
}

// Kalman Filter Implementation
void kalmanFilter(float accelAngle, float gyroRate, float dt) {
  // Step 1: Predict
  pitch += dt * (gyroRate - bias);  // Update pitch estimate using gyroscope rate
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  // Step 2: Update
  float y = accelAngle - pitch;  // Angle difference
  float S = P[0][0] + R_measure;  // Estimate error
  float K[2];  // Kalman gain
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  // Adjust pitch and bias
  pitch += K[0] * y;
  bias += K[1] * y;

  // Update the error covariance matrix
  float P00_temp = P[0][0];
  float P01_temp = P[0][1];

  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;
}

// Function to calibrate pitch offset (only when sensor is level)
void calibratePitch() {
  sensors_event_t accel, temp;

  // Read accelerometer data
  mpu.getEvent(&accel, &temp, &temp);

  // Calculate the pitch angle from accelerometer data
  float accelPitch = atan2(accel.acceleration.y, accel.acceleration.z) * 180 / PI;

  // Check if the sensor is near level (within a small tolerance)
  if (fabs(accelPitch) < 5.0) {  // 5 degrees tolerance for being level
    // If near level, set the pitch offset as the calibration value
    pitchOffset = accelPitch;

    // If the sensor is reading near -0.4 degrees on flat ground, we can set pitchOffset to 0.4 to correct
    if (pitchOffset < 0) {
      pitchOffset = 0;  // Ensure we don't have a negative offset
    }

    Serial.print("Measured Pitch Bias: ");
    Serial.println(pitchOffset);
    Serial.print("Calibrated Pitch Offset: ");
    Serial.println(pitchOffset);

  } else {
    Serial.println("Sensor not level. Skipping calibration.");
  }
}
