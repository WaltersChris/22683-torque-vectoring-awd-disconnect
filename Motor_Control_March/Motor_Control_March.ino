#include <AccelStepper.h>  // AccelStepper library to control the stepper motor
#include <MultiStepper.h>

// Stepper motor driver settings:
// Micro Step setting (SW1~4): 200, 40000 pulses/rev
// Rotation (SW5): CCW
// Control Mode (SW6): Closed Loop
// Pulse Mode (SW7): PUL/DIR
// Pulse Filter Time (SW8): No

// Define stepper motor control pins
#define STEP_PIN D2   // Pin for step pulse (STEP+)
#define DIR_PIN D3    // Pin for direction (DIR+)
#define ENA_PIN D4    // Optional pin for enable (ENA+)

// Definitions for LEDs
#define OverTemp_LED D5 // Red overtemperature LED
#define TwoWheel_LED D6 // Yellow 2WD LED
#define FourWheel_LED D7 // Green 4WD LED

// Additional definitions
#define MotorAlarm_PIN D8 // optional Motor Driver over voltage or over current monitoring

// Selection definitions - Analog Pins
#define SystemOn_PIN A0 // Selection for system on/off
#define MuSelect_PIN A1 // Selection for Mu/Friction Coefficient
#define SystemStateSelect_Pin A2 // Selection for 2WD, 4WD, or 4WD Lock

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
  //{x0, x0, xx0},
   //Add more intervals as needed
}; 
  // This might not be how we do things

// 
   //int SystemState = 0; //  Place to store 2WD or 4WD state. 0 = 2WD, 1 = 4WD, 2 = 4WD Lock
  // Need way to convert analog value to this digital scheme

void setup() {

  Serial.begin(115200);
  while (!Serial) delay(10);
//
//  // Initialize stepper motor parameters
//  stepper.setMaxSpeed(1000);    // Set maximum speed (adjust as needed)
//  stepper.setAcceleration(500); // Set acceleration (adjust as needed)
//
//  // enable/disable pin
//  pinMode(ENA_PIN, OUTPUT);
//  digitalWrite(ENA_PIN, HIGH); // Enable motor (set LOW to disable if ENA is used)
}

void loop() {

    SystemState = analogRead(SystemStateSelect_Pin); // Read "SystemStateSelect_Pin" to determine 2WD or 4WD
    if (SystemState == 0){
      // do nothing (2WD)
      // delay for polling input
    }
    






  
//  // Hardcoded torque value 
//  float torqueInput = 54.0;  // Example torque input (in Nm)
//
//  // Call the function to calculate the motor rotation angle based on torque input
//  int targetDegrees = mapTorqueToDegrees(torqueInput);
//
//  // Rotate the stepper motor to the calculated angle
//  stepper.moveTo(targetDegrees);
//  stepper.runToPosition();  // Move to the target position 
//}
//
//// Torque-to-Degrees Mapping Function
//int mapTorqueToDegrees(float torque) {
//  for (int i = 0; i < sizeof(torqueIntervals) / sizeof(torqueIntervals[0]); i++) {
//    if (torque >= torqueIntervals[i].minTorque && torque < torqueIntervals[i].maxTorque) {
//      return torqueIntervals[i].degrees;
//    }
//  }
//  return 0;  // Return 0 if torque doesn't fall in any defined range
}
