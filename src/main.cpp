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

float Kp = 0.05;
//float Ki = 0.00009;
float Ki = 0;
//float Kd = 0.055;
float Kd = 0;
int P;
int I = 0;
int D;
int lastError = 0;

const uint8_t maxspeed = 100;
const uint8_t rightBaseSpeed = 57;
const uint8_t leftBaseSpeed = 60;

Motor right_motor(IN_1A, IN_2A, PWM_A, offsetA, 18);
Motor left_motor(IN_1B, IN_2B, PWM_B, offsetB, 18);

void setup()
{

Serial.begin(9600);
// configure the sensors
qtr.setTypeAnalog();
qtr.setSensorPins((const uint8_t[]){S1, S2, S3, S4, S5}, SensorCount);
qtr.setEmitterPin(IR);


//calibration
for (uint16_t i = 0; i < 100; i++)
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
  float error = 2000 - position;
  //float error = (-1.5 * sensorValues[0] -1.2 * sensorValues[1] + 1 * sensorValues[3] + 1 * sensorValues[4]);

  //Serial.print("Sum: ");
  //Serial.println(sum);

  P = error;
  I += error;
  D = error - lastError;
  lastError = error;
  int motorspeed = P * Kp + I * Ki + D * Kd;

  int left_motorspeed = leftBaseSpeed + motorspeed;
  int right_motorspeed = rightBaseSpeed - motorspeed;

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


void robot_forward(){

  right_motor.drive(rightBaseSpeed);
  left_motor.drive(leftBaseSpeed); 

}


void loop()
{

  PID_control();
  //robot_forward();
  // Imprimir os valores dos sensores para depuração

  uint16_t position = qtr.readLineBlack(sensorValues);
  
  for (uint8_t i = 0; i < SensorCount; i++) 
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);
}
