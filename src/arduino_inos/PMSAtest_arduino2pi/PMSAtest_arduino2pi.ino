// 
// This is a test for the PMSA003i PM2.5 sensor paired with the Adafruit Feather Board 
// I intend to use this code to learn how communication between the arduino microcontroller and the raspberry pi 5 works

#include <Wire.h>
#include <Adafruit_PM25AQI.h>

// Create an instance of the sensor
Adafruit_PM25AQI aqiSensor;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10); // Wait for Serial to initialize

  Serial.println("PMSA003i Air Quality Sensor Test");

  // Initialize the sensor
  if (!aqiSensor.begin_I2C()) { // Using default I²C address
    Serial.println("Could not find PMSA003i sensor! Check wiring.");
    while (1) delay(10);
  }
  Serial.println("PMSA003i found!");
}

void loop() {
  PM25_AQI_Data data;

  // Request data from the sensor
  if (aqiSensor.read(&data)) {
    Serial.println("PM2.5 Air Quality Data:");
    Serial.print("PM 1.0: "); Serial.print(data.pm10_standard); Serial.println(" µg/m³");
    Serial.print("PM 2.5: "); Serial.print(data.pm25_standard); Serial.println(" µg/m³");
    Serial.print("PM 10: "); Serial.print(data.pm100_standard); Serial.println(" µg/m³");
    Serial.println();

    delay(2000); // Wait 2 seconds before reading again
  } else {
    Serial.println("Failed to read from PMSA003i sensor!");
    delay(500); // Wait and try again
  }
}
 