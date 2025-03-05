/*********
  Rui Santos
  Complete project details at http://randomnerdtutorials.com  
  Based on the Dallas Temperature Library example
*********/

#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is connected to the Arduino digital pin 4
#define ONE_WIRE_BUS 4

// Red LED pin connected to digital pin 8
#define RED_LED_PIN 8

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

void setup(void)
{
  // Start serial communication for debugging purposes
  Serial.begin(9600);
  
  // Initialize the Dallas Temperature library
  sensors.begin();
  
  // Set the LED pin as output
  pinMode(RED_LED_PIN, OUTPUT);
  
  // Ensure the LED is off at the start
  digitalWrite(RED_LED_PIN, LOW);
}

void loop(void)
{ 
  // Request temperature readings from all devices on the bus
  sensors.requestTemperatures(); 
  
  // Read the temperature in Fahrenheit from the first sensor
  float tempF = sensors.getTempFByIndex(0);

  // Print the Fahrenheit temperature reading
  Serial.print("Fahrenheit temperature: ");
  Serial.println(tempF);
  
  // Check if the temperature exceeds 100°F
  if (tempF > 140) {
    // Turn on the red LED
    digitalWrite(RED_LED_PIN, HIGH);
  } else {
    // Turn off the red LED
    digitalWrite(RED_LED_PIN, LOW);
  }

  // Wait for 1 second before the next reading
  delay(1000);
}
