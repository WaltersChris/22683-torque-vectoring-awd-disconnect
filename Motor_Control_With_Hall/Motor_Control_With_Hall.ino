#include <AccelStepper.h>  // AccelStepper library to control the stepper motor
//#include <MultiStepper.h>

// Stepper motor driver settings:
// Micro Step setting (SW1~4): 1000 pulses/rev
// Rotation (SW5): CW
// Control Mode (SW6): Closed Loop
// Pulse Mode (SW7): PUL/DIR
// Pulse Filter Time (SW8): Yes

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

// Hall Effect
#define HALL_SENSOR_PIN 13   
#define MAGNETS 14          
#define MEASURE_INTERVAL 100  // 100ms updates 

  volatile int pulseCount = 0;
  unsigned long lastPulseTime = 0;
  float rpmFiltered = 0;
  float rpmRaw = 0;

  void hallSensorISR() {
      static unsigned long lastInterruptTime = 0;
      unsigned long interruptTime = micros();

      if (interruptTime - lastInterruptTime > 1000) {  // 1ms debounce
          pulseCount++;
          lastInterruptTime = interruptTime;

          if (lastPulseTime > 0) {  // Only compute if we have at least 1 pulse
              float timePerPulse = (interruptTime - lastPulseTime) / 1e6; // Convert µs to s
              rpmRaw = (60.0 / (timePerPulse * MAGNETS));  //  RPM calculation
          }
          lastPulseTime = interruptTime;
      }
  }

  int clutch_Kiss = 0;
  int clutch_Calibrated = 0;

// Define the stepper motor
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);  // Using a stepper driver 

// Torque-to-angle mapping: Define the torque intervals and corresponding degrees
struct TorqueInterval {
  float minTorque;
  float maxTorque;
  int degrees;
};

int Forward = 2;
int Reverse = 2; // Test code, user inputs number of pulses to go forward and reverse with the motor

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
  stepper.setMaxSpeed(200);
  stepper.setAcceleration(750);
  stepper.moveTo(0); // 500 is  full rev at 1k pulse/rev

  Serial.begin(115200);
  while (!Serial) delay(10);
  pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hallSensorISR, RISING); 
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
      unsigned long currentTime = millis();
    
    if (currentTime - lastPulseTime > MEASURE_INTERVAL) { 
        noInterrupts();
        int count = pulseCount;
        pulseCount = 0;
        interrupts();

        
        rpmFiltered = (rpmFiltered * 0.7) + (rpmRaw * 0.3);

        Serial.print("Pulses: ");
        Serial.print(count);
        Serial.print(" | Raw RPM: ");
        Serial.print(rpmRaw);
        Serial.print(" | Filtered RPM: ");
        Serial.println(rpmFiltered);
    }
    

//       stepper.setSpeed(100);  
//       stepper.runSpeed();    // Start moving motor forward
       
    if (rpmFiltered <= 10)  { //If RPM is greater than 0 (or a low amount)
//      stepper.stop(); // Stop motor when clutch is engaged
      clutch_Kiss = 1; // Set varible to clutch engaged
      digitalWrite(OverTemp_LED, HIGH) // test code for no motor
    }
    else if (clutch_Kiss = 1) {
//       stepper.setSpeed(-50);  
//       stepper.runSpeed(); // Start backing up motor

        if (rpmFiltered >= 10){ // Clutch no longer engaged          
//       stepper.stop();
          clutch_Calibrated = 1; // Set calibration varible
          digitalWrite(TwoWheel_LED, HIGH) // test code for no motor
        }
    }

  
        
 //If at the end of travel go to the other end
//    if (stepper.distanceToGo() == 0)
//      stepper.moveTo(-stepper.currentPosition());
//   stepper.runToNewPosition(10);
//   stepper.run();
   // digitalWrite(DIR_PIN, LOW);
//    stepper.runToPosition();
//        delay(10);
//    Serial.print('x');
//    delay(500);
    //digitalWrite(DIR_PIN, HIGH);
//  while (Serial.available() > 0) {    
//    Serial.print('?');
//        delay(5);
//    Forward = Serial.parseInt();
//        delay(5);
//    Forward = 100*Forward;
//    stepper.moveTo(Forward);
//       delay(5);
//    stepper.runToPosition();
//    Serial.print('F');
//        delay(5);
////    delay(500);
//    Serial.print('?');
//        delay(5);
//    Reverse = Serial.parseInt();
//        delay(5);
//    Reverse = 100*Reverse;  
//    stepper.moveTo(-Reverse);
////    digitalWrite(DIR_PIN, HIGH);
//
//    stepper.runToPosition();
//    Serial.print('R');
//    delay(5);
  }
            // ~~Test code~~



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

//// Degrees-to-Pulses Mapping Function


// Torque Calculation Function

// Function for overtemp LED

}
