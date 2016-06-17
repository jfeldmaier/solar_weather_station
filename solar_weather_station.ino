#include <Wire.h>
#include "Adafruit_AM2315.h"

/*
 * Solar powered weather station
 * 
 * The solar cell is connected to Seeedstudio's Solar 
 * charger shield V2.0. The battery voltage can be 
 * measured via Analog Pin 0 (A0). 
 * 
 * An outdoor Humidity and Temoerature Sensor 
 * (Aosong AM2315) is connected via i2c. Use following 
 * wiring:
 * 
 * RED of the AM2315 sensor to 5.0V
 * BLACK to Ground
 * WHITE to i2c clock - on Leonardo to Pin 3 (SCL)
 * YELLOW to i2c data - on Leonardo to Pin 2 (SDA)
 * 
 */

Adafruit_AM2315 am2315;

#define statusLed 13
#define batteryPin A0

#define DEBUG 1

void setup() {
  delay(500); // The AM2315 needs some time to come up

  if (DEBUG) {
    Serial.begin(57600);
    Serial.println("Solar Weather Station has started");
  }

  pinMode(statusLed, OUTPUT);

  while(!am2315.begin()) {
    if (DEBUG) {
      Serial.println("Sensor not found, check wiring & pullups!");
    }
     blinkFast();
     delay(500); 
  }
  

}

void loop() {
  float humidity = am2315.readHumidity();
  float outTemp = am2315.readTemperature();
  float batteryVoltage = readBatteryVoltage();

  if (DEBUG) {
    Serial.print("Hum: ");
    Serial.print(humidity);
    Serial.print(" outTemp: ");
    Serial.println(outTemp);
    Serial.print("Battery Voltage: ");
    Serial.println(batteryVoltage);
  }

  blink();
  delay(1000);
  
}

float readBatteryVoltage() {
  int rawVal = analogRead(batteryPin);
  // TODO: implement averaging
  return (float(rawVal)*5)/1023*2;
}

void blinkFast() {
  digitalWrite(statusLed,1);
  delay(100);
  digitalWrite(statusLed,0);
}

void blink() {
  digitalWrite(statusLed,1);
  delay(400);
  digitalWrite(statusLed,0);
}
