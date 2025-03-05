#define HALL_SENSOR_PIN 2   
#define MAGNETS 14          
#define MEASURE_INTERVAL 100  // Reduced from 500ms to 100ms for faster updates

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
            rpmRaw = (60.0 / (timePerPulse * MAGNETS));  // Instantaneous RPM calculation
        }
        lastPulseTime = interruptTime;
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hallSensorISR, RISING);  // Try RISING instead of FALLING
}

void loop() {
    unsigned long currentTime = millis();
    
    if (currentTime - lastPulseTime > MEASURE_INTERVAL) { 
        noInterrupts();
        int count = pulseCount;
        pulseCount = 0;
        interrupts();

        // Smoothed RPM using Exponential Moving Average
        rpmFiltered = (rpmFiltered * 0.7) + (rpmRaw * 0.3);

        Serial.print("Pulses: ");
        Serial.print(count);
        Serial.print(" | Raw RPM: ");
        Serial.print(rpmRaw);
        Serial.print(" | Filtered RPM: ");
        Serial.println(rpmFiltered);
    }
}
