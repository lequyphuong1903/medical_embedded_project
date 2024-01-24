#include <Arduino.h>
#include <Wire.h>
#include "MAX30105.h"
#define TIMEOUT_COUNT 1000
#define _vo                     volatile



bool _vo OnMeasurement = false;



MAX30105 particleSensor;
byte interruptPin = 3;
byte EnBLE_PIN = 7;
byte device_id = 1;
const int BufferLength = 5;
uint16_t irBuffer[BufferLength];
uint16_t redBuffer[BufferLength];
uint8_t data[sizeof(irBuffer) + sizeof(redBuffer) + sizeof(byte)];
int samplesTaken = 0;
uint8_t startCOMMAND[] = {1,2,3,4,5,6};
uint8_t stopCOMMAND[] = {2,5,2,3,4,6};



void USART_Init() 
{
  UBRR0H = (8>>8);
  UBRR0L = 8;
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void USART_Transmit(uint8_t* data)
{
  for (uint8_t i = 0; i < 21; i++)
  {
    while (!(UCSR0A & (1<<UDRE0)));
    UDR0 = data[i];
  }
}

uint8_t* USART_Receive(void)
{
  uint8_t *arr = (uint8_t*)malloc(6);
  for (uint8_t i = 0; i < 6; i++)
  {
    while (!(UCSR0A & (1<<RXC0)));
    arr[i] = UDR0;
  }
  return arr;
}

bool checkCOMMAND(uint8_t *arr1, uint8_t *arr2) {
  for (uint8_t i = 0; i < 6; i++)
  {
    if (arr1[i] != arr2[i])
    {
      return 0;
    }
  }
  return 1;
}

void setup()
{
  pinMode(interruptPin, INPUT);
  pinMode(EnBLE_PIN, OUTPUT);
  USART_Init();
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    while (1);
  }
  

  byte ledBrightness = 0xFF; //Options: 0=Off to 255=50mA
  byte sampleAverage = 1; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int sampleRate = 800; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 8192; //Options: 2048, 4096, 8192, 16384
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  particleSensor.enableAFULL();
  particleSensor.setFIFOAlmostFull(3);
}

void loop()
{
  particleSensor.check();
  while (particleSensor.available())
  {
    memcpy(data, &device_id, sizeof(byte));
    for(uint8_t i = 0; i < BufferLength; i++){
      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample();
    }
    memcpy(data + sizeof(byte), (uint8_t*)redBuffer, sizeof(redBuffer));
    memcpy(data + sizeof(redBuffer) + sizeof(byte), (uint8_t*)irBuffer, sizeof(irBuffer));
    USART_Transmit(data);
  }
}