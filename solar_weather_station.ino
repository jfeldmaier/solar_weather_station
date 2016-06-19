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
 * NRF24 Connection (specific for Arduino Leonardo)
 * GND - GND / VCC - 3,3V
 * NRF24 - Leonardo
 * CE (Pin 3) - 9
 * CSN (Pin4) - 10
 * SCK (Pin 5) - ICSP SCK
 * MOSI - ICSP MOSI
 * MISO - ICSP MISO
 * 
 * Note: For compiling MySensors Lib it is necessary to comment out 
 *       line 6 (USBDevice.init()) in "MyMainDefault.cpp". 
 * 
 */

#define statusLed 13
#define batteryPin A0

#define DEBUG 1
//#define MY_DEBUG // Enable/Disable debug outputs of the MySensors Lib

// MySensors Setup
#define MY_RADIO_NRF24
#define MY_NODE_ID 20
#define CHILD_ID_HUM 0
#define CHILD_ID_TEMP 1

// Following libs must be included in local lib folder
#include <Wire.h>
#include <SPI.h>
#include <MySensor.h>
// Local libs in sketch folder
#include "Adafruit_AM2315.h"

Adafruit_AM2315 am2315;

boolean metric = true; 
MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);


void setup() {
  // Setup MySensors
  metric = getConfig().isMetric;

  
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

void presentation()  
{ 
  
  // Send the Sketch Version Information to the Gateway
  sendSketchInfo("Outdoor Weather Station", "0.1");

  // Register all sensors to gw (they will be created as child devices)
  present(CHILD_ID_HUM, S_HUM);
  present(CHILD_ID_TEMP, S_TEMP);
}

void loop() {
  float humidity = am2315.readHumidity();
  float outTemp = am2315.readTemperature();
  float batteryVoltage = readBatteryVoltage();
  int batteryPcnt = 0;
  // TODO: calculate proper percentage value; also some averaging is necessary
  batteryPcnt = (batteryVoltage - 3.0) * 83.33;

  if (DEBUG) {
    Serial.print("Hum: ");
    Serial.print(humidity);
    Serial.print(" outTemp: ");
    Serial.println(outTemp);
    Serial.print("Battery Voltage: ");
    Serial.println(batteryVoltage);
  }

  // Send Measurements
  sendBatteryLevel(batteryPcnt);
  send(msgTemp.set(outTemp, 1));
  send(msgHum.set(humidity, 1));

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
