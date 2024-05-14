#include <Arduino.h>
#include "pins.h"
#include "SparkFun_TB6612.h"
#include <QTRSensors.h>

volatile long right_pulseCounter = 0;
volatile long left_pulseCounter = 0;
volatile unsigned long timeOld;
const unsigned int pulsesPerRotation = 20;
const int offsetA = 1;
const int offsetB = 1;


QTRSensors qtr;
const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];

float Kp = 0.12;
float Ki = 0;
float Kd = 0.055;
int P;
int I;
int D;
int lastError = 0;

const uint8_t maxspeed = 100;
const uint8_t basespeed = 60;

Motor right_motor(IN_1A, IN_2A, PWM_A, offsetA, 18);
Motor left_motor(IN_1B, IN_2B, PWM_B, offsetB, 18);

/*CONTADORES DO ENCODER

int right_speed = 0;
int left_speed = 0;

void incrementACounter()
{
  // Incrementa contador do motor A
  if (speedA < 0) pulseCounterA--;
  else pulseCounterA++;
}

void incrementBCounter() ]
{
  // Incrementa contador do motor B
  if (speedB < 0) pulseCounterB--;
  else pulseCounterB++;
} */

void setup()
{

Serial.begin(9600);
// configure the sensors
qtr.setTypeAnalog();
qtr.setSensorPins((const uint8_t[]){S1, S2, S3, S4, S5}, SensorCount);
qtr.setEmitterPin(IR);

/*pinMode(ENCODER_A_PIN, INPUT);
attachInterrupt(ENCODER_A_PIN, incrementACounter, FALLING); encoder stuff*/

//calibration
for (uint16_t i = 0; i < 300; i++)
  {
    qtr.calibrate();
  }

for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);

}

void PID_control() 
{
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = 1500 - position;

  P = error;
  I += error;
  D = error - lastError;
  lastError = error;
  int motorspeed = P * Kp + I * Ki + D * Kd;

  int left_motorspeed = basespeed + motorspeed;
  int right_motorspeed = basespeed - motorspeed;

  // Limite a velocidade dos motores
  if (right_motorspeed > maxspeed) 
  {
    right_motorspeed = maxspeed;
  }
  if (right_motorspeed < 0) 
  {
    right_motorspeed = 0;
  }
  if (left_motorspeed > maxspeed) 
  {
    left_motorspeed = maxspeed;
  }
  if (left_motorspeed < 0) 
  {
    left_motorspeed = 0;
  }
 

  right_motor.drive(right_motorspeed);
  left_motor.drive(left_motorspeed);
}


void loop()
{
  PID_control();
  
  // Imprimir os valores dos sensores para depuração

  uint16_t position = qtr.readLineBlack(sensorValues);
  for (uint8_t i = 0; i < SensorCount; i++) 
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);
}