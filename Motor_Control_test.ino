#include <AccelStepper.h>  // AccelStepper library to control the stepper motor

// Define stepper motor control pins
#define STEP_PIN 3   // Pin for step pulse (STEP+)
#define DIR_PIN 2    // Pin for direction (DIR+)
#define ENA_PIN 4    // Optional pin for enable (ENA+)

// Define the stepper motor
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);  // Using a stepper driver 

// Torque-to-angle mapping: Define the torque intervals and corresponding degrees
struct TorqueInterval {
  float minTorque;
  float maxTorque;
  int degrees;
};

TorqueInterval torqueIntervals[] = {
  {50, 60, 200},   // Torque between 50-60 Nm corresponds to 200 degrees
  {60, 70, 220},   // Torque between 60-70 Nm corresponds to 220 degrees
  {70, 80, 240},   // Torque between 70-80 Nm corresponds to 240 degrees
  // Add more intervals as needed
};

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Initialize stepper motor parameters
  stepper.setMaxSpeed(1000);    // Set maximum speed (adjust as needed)
  stepper.setAcceleration(500); // Set acceleration (adjust as needed)

  // enable/disable pin
  pinMode(ENA_PIN, OUTPUT);
  digitalWrite(ENA_PIN, HIGH); // Enable motor (set LOW to disable if ENA is used)
}

void loop() {
  // Hardcoded torque value 
  float torqueInput = 54.0;  // Example torque input (in Nm)

  // Call the function to calculate the motor rotation angle based on torque input
  int targetDegrees = mapTorqueToDegrees(torqueInput);

  // Rotate the stepper motor to the calculated angle
  stepper.moveTo(targetDegrees);
  stepper.runToPosition();  // Move to the target position 
}

// Torque-to-Degrees Mapping Function
int mapTorqueToDegrees(float torque) {
  for (int i = 0; i < sizeof(torqueIntervals) / sizeof(torqueIntervals[0]); i++) {
    if (torque >= torqueIntervals[i].minTorque && torque < torqueIntervals[i].maxTorque) {
      return torqueIntervals[i].degrees;
    }
  }
  return 0;  // Return 0 if torque doesn't fall in any defined range
}
