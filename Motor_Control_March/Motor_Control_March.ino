#include <AccelStepper.h>  // AccelStepper library to control the stepper motor
#include <MultiStepper.h>

// Stepper motor driver settings:
// Micro Step setting (SW1~4): 200, 40000 pulses/rev
// Rotation (SW5): CCW
// Control Mode (SW6): Closed Loop
// Pulse Mode (SW7): PUL/DIR
// Pulse Filter Time (SW8): No

// Define stepper motor control pins
#define STEP_PIN 2   // Pin for step pulse (STEP+)
#define DIR_PIN 3    // Pin for direction (DIR+)
#define ENA_PIN 4    // Optional pin for enable (ENA+)

// Definitions for LEDs
#define OverTemp_LED 5 // Red overtemperature LED
#define TwoWheel_LED 6 // Yellow 2WD LED
#define FourWheel_LED 7 // Green 4WD LED

// Additional definitions
#define MotorAlarm_PIN 8 // optional Motor Driver over voltage or over current monitoring
#define HallEffect_PIN 9 // Digital? readout for the hall effect sensor

// Selection definitions - Analog Pins
#define SystemOn_PIN A0 // Selection for system on/off
#define MuSelect_PIN A1 // Selection for Mu/Friction Coefficient
#define SystemStateSelect_PIN A2 // Selection for 2WD, 4WD, or 4WD Lock
#define TempSensor_PIN A3 // Analog readout for the temperature sensor


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
  
  //{x0, x0, xx0}, // Full lock: around 270 deg
   //Add more intervals as needed
}; 
  // This might not be how we do things

  //int SystemState = 0; //  Place to store 2WD or 4WD state. 0 = 2WD, 1 = 4WD, 2 = 4WD Lock
  // Need way to convert analog value to this digital scheme

void setup() {
  stepper.setMaxSpeed(300);
  stepper.setAcceleration(750);
  stepper.moveTo(0); // 500 is  full rev at 1k pulse/rev

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

// If at the end of travel go to the other end
//    if (stepper.distanceToGo() == 0)
//      stepper.moveTo(-stepper.currentPosition());
//   stepper.runToNewPosition(10);
//   stepper.run();
//   stepper.runToNewPosition(10);
    stepper.run();
    stepper.moveTo(10);
    stepper.run();
    stepper.moveTo(250);
        stepper.run();
    stepper.moveTo(-250);
        stepper.run();
    stepper.moveTo(500);
//   stepper.runToNewPosition(10);

//    SystemState = analogRead(SystemStateSelect_Pin); // Read "SystemStateSelect_Pin" to determine 2WD or 4WD
//    if (SystemState == 0){ // 2WD state
      // do nothing (2WD)
      // delay for polling input
//    }
//    else if (SystemState == 1){ // 4WD
      // use the move to target degree routine below?
//    }
//    else if (SystemState == 2){ // 4WD Lock
      // use the move to target degree with deg = max

//    }

  
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

// Torque Calculation Function

// Function for overtemp LED

}
