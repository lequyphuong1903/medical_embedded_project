#include <Arduino.h>
#include <Wire.h>
#include "MAX30105.h"

MAX30105 particleSensor;
byte interruptPin = 3; //Connect INT pin on breakout board to pin 3
byte device_id = 6;
byte sendBuffer = 0;
const uint32_t BufferLength = 50;
uint32_t irBuffer[BufferLength];
uint32_t redBuffer[BufferLength];
byte data[sizeof(irBuffer) + sizeof(redBuffer) + sizeof(byte)*2];

void setup()
{
  pinMode(interruptPin, INPUT);
  Serial.begin(115200);
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }

  byte ledBrightness = 0x0F; //Options: 0=Off to 255=50mA
  byte sampleAverage = 1; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int sampleRate = 400; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 69; //Options: 69, 118, 215, 411
  int adcRange = 2048; //Options: 2048, 4096, 8192, 16384
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  particleSensor.enableAFULL();
  particleSensor.setFIFOAlmostFull(2);
}

void loop()
{
  sendBuffer = Serial.read();
  particleSensor.check();
  while (particleSensor.available()) //do we have new data?
  {
    
    byte temperature = (byte)particleSensor.readTemperature();
    memcpy(data, &device_id, sizeof(byte));
    memcpy(data + sizeof(byte), &temperature, sizeof(byte));
    for(uint8_t i = 0; i < BufferLength; i++) {
      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample();
    }
    memcpy(data + sizeof(byte)*2, (byte*)redBuffer, sizeof(redBuffer));
    memcpy(data + sizeof(redBuffer) + sizeof(byte)*2, (byte*)irBuffer, sizeof(irBuffer));
  }
  if (sendBuffer == device_id)
  {
      Serial.write(data, sizeof(data));
      delay(10);
  }
}