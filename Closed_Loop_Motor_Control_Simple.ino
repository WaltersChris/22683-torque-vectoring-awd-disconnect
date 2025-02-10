// Define motor driver pins
#define STEP_PIN 3  // Pin for step pulse (STEP+)
#define DIR_PIN 2   // Pin for direction (DIR+)
#define ENA_PIN 4   // Optional pin for enable (ENA+)

// Encoder pins
#define ENCODER_A 5  // Encoder A+ pin
#define ENCODER_B 6  // Encoder B+ pin

// Global variables for encoder
volatile int encoderPosition = 0;  // Track encoder position
int lastEncoderPosition = 0;  // For smooth control and tracking changes

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

// Encoder debounce settings
unsigned long lastDebounceTime = 0;  // Timestamp for last debounce check
unsigned long debounceDelay = 10;    // Time to debounce in milliseconds

// Interrupt Service Routine to track encoder position
void encoderISR() {
  // Read encoder pins
  int A = digitalRead(ENCODER_A);
  int B = digitalRead(ENCODER_B);

  // Simple debounce logic
  if (millis() - lastDebounceTime > debounceDelay) {
    if (A == B) {
      encoderPosition++;  // Clockwise direction
    } else {
      encoderPosition--;  // Counter-clockwise direction
    }
    lastDebounceTime = millis();  // Update last debounce time
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Initialize motor control pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  digitalWrite(ENA_PIN, HIGH);  // Enable motor

  // Set encoder pins
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);

  // Attach interrupts for encoder A and B signals
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoderISR, CHANGE);

  // Initialize motor position
  encoderPosition = 0;
}

void loop() {
  // Hardcoded torque value
  float torqueInput = 54.0;  

  // Map torque to degrees
  int targetDegrees = mapTorqueToDegrees(torqueInput);

  // Convert degrees to steps 
  int targetSteps = degreesToSteps(targetDegrees);

  // Move motor to the target position
  moveToTargetPosition(targetSteps);
}

// Map torque to corresponding degrees
int mapTorqueToDegrees(float torque) {
  for (int i = 0; i < sizeof(torqueIntervals) / sizeof(torqueIntervals[0]); i++) {
    if (torque >= torqueIntervals[i].minTorque && torque < torqueIntervals[i].maxTorque) {
      return torqueIntervals[i].degrees;
    }
  }
  return 0;  // Return 0 if torque doesn't fall in any defined range
}

// Convert degrees to steps 
int degreesToSteps(int degrees) {
  // 1.8° per step (200 steps per full revolution)
  int stepsPerRevolution = 200;  
  return (stepsPerRevolution * degrees) / 360;
}

// Move motor to the target position based on encoder feedback
void moveToTargetPosition(int targetSteps) {
  int currentSteps = encoderPosition;  // Read current encoder position

  // Set motor direction based on target position
  if (targetSteps > currentSteps) {
    digitalWrite(DIR_PIN, HIGH);  // Set direction to clockwise
  } else {
    digitalWrite(DIR_PIN, LOW);   // Set direction to counter-clockwise
  }

  // Move motor until it reaches the target
  while (currentSteps != targetSteps) {
    digitalWrite(STEP_PIN, HIGH);  // Create a step pulse

    // Adjust step pulse timing 
    int stepDelay = 500 - (abs(targetSteps - currentSteps) / 2);  // Faster as it gets closer to the target
    stepDelay = max(stepDelay, 100);  // Set a lower bound for speed (to avoid too fast)

    delayMicroseconds(stepDelay);  // Control speed

    digitalWrite(STEP_PIN, LOW);   // Complete step pulse
    delayMicroseconds(stepDelay);  // Wait for the next pulse

    // Read current encoder position
    currentSteps = encoderPosition;

    // Small delay to avoid over-polling the encoder
    delay(1);  // 1 ms delay, adjust for smoother motion
  }

  // Stop the motor when target is reached
  digitalWrite(STEP_PIN, LOW);
  delay(500);  // Delay after movement

}
