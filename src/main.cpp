#include <Arduino.h>
#include <Wire.h>
#include "MAX30105.h"

MAX30105 particleSensor;
byte interruptPin = 3;
byte EnBLE_PIN = 7;
byte device_id = 1;
const int BufferLength = 2;
uint16_t irBuffer[BufferLength];
uint16_t redBuffer[BufferLength];
byte data[sizeof(irBuffer) + sizeof(redBuffer) + sizeof(byte)];
int samplesTaken = 0;
void setup()
{
  pinMode(interruptPin, INPUT);
  pinMode(EnBLE_PIN, OUTPUT);
  Serial.begin(115200);
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  byte ledBrightness = 0x6F; //Options: 0=Off to 255=50mA
  byte sampleAverage = 1; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int sampleRate = 200; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 8192; //Options: 2048, 4096, 8192, 16384
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  particleSensor.enableAFULL();
  particleSensor.setFIFOAlmostFull(3);
}

void loop()
{ 
  particleSensor.check();
  while (particleSensor.available()) //do we have new data?
  {
    memcpy(data, &device_id, sizeof(byte));
    for(uint8_t i = 0; i < BufferLength; i++) {
      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample();
    }
    memcpy(data + sizeof(byte), (byte*)redBuffer, sizeof(redBuffer));
    memcpy(data + sizeof(redBuffer) + sizeof(byte), (byte*)irBuffer, sizeof(irBuffer));
    Serial.write(data, sizeof(data));
  }
}