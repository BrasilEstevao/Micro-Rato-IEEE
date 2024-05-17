#include <Arduino.h>
#include "config.h"
#include "SparkFun_TB6612.h"
#include <QTRSensors.h>

volatile long right_pulseCounter = 0;
volatile long left_pulseCounter = 0;
volatile unsigned long timeOld;
const unsigned int pulsesPerRotation = 20;
const int offsetA = 1, offsetB = 1;

byte state;
uint32_t tis, tes;

QTRSensors qtr;
const uint8_t SensorCount = 4;
uint16_t sensorValues[SensorCount];

float Kp = 0.05;
float Ki = 0.0002;
//float Ki = 0;
//float Kd = 0.055;
float Kd = 0;
int P;
int I = 0;
int D;
int lastError = 0;


const uint8_t maxspeed = 255;
const uint8_t rightBaseSpeed = 57;
const uint8_t leftBaseSpeed = 60;

Motor right_motor(IN_1A, IN_2A, PWM_A, offsetA, 18);
Motor left_motor(IN_1B, IN_2B, PWM_B, offsetB, 18);

//=========================================
//Function Headers

void followLine();
void setState(byte new_state);
void robot_forward();
void setRobotVW(float V, float W);
void readSensors();
void calcCrosses();
void statesEvolution();
void statesExits();

//=========================================

int motorspeed = 0;
int crosses = 0;
char decisionFlag = 'N';

void setup(){

setState(99);
//setState(start);


Serial.begin(9600);
// configure the sensors
//qtr.setTypeRC();
qtr.setTypeAnalog();
qtr.setSensorPins((const uint8_t[]){S1, S2, S3, S4}, SensorCount);
qtr.setEmitterPin(IR);


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

  //delay(2000);

}

void loop()
{

  statesEvolution();
  statesExits();

  //followLine();

  calcCrosses();


  //PID_control();
  //robot_forward();
  // Imprimir os valores dos sensores para depuração

  uint16_t position = qtr.readLineBlack(sensorValues);
  //qtr.read(sensorValues);

  //readSensors();
  
  for (uint8_t i = 0; i < SensorCount; i++) 
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  //Serial.println();
  Serial.print(position);
  Serial.print('\t');

  Serial.print("Crosses: ");
  Serial.println(crosses);

}


//===================================
//Funções

void followLine(){
  uint16_t position = qtr.readLineBlack(sensorValues);
  float error = 1400 - position;
  //float error = (-1.5 * sensorValues[0] -1.2 * sensorValues[1] + 1 * sensorValues[3] + 1 * sensorValues[4]);
  //float error = (-1 * sensorValues[1] + 1 * sensorValues[2]);
  //Serial.print("Sum: ");
  //Serial.println(sum);

  P = error;
  I += error;
  D = error - lastError;
  lastError = error;
  motorspeed = P * Kp + I * Ki + D * Kd;

  int left_motorspeed = leftBaseSpeed + motorspeed;
  int right_motorspeed = rightBaseSpeed - motorspeed;

  constrain(right_motorspeed, 0, maxspeed);
  constrain(left_motorspeed, 0, maxspeed); 

  right_motor.drive(right_motorspeed);
  left_motor.drive(left_motorspeed);
}

void setState(byte new_state){
  tes = millis();
  tis = 0;
  state = new_state;
}

void robot_forward(){

  right_motor.drive(rightBaseSpeed);
  left_motor.drive(leftBaseSpeed); 

}

void setRobotVW(float V, float W){
  int leftDuty = map(V - W, 0, 10, 0, 255);
  int rightDuty = map(V + W, 0, 10, 0, 255);

  left_motor.drive(leftDuty);
  right_motor.drive(rightDuty);

}

void readSensors(){
  qtr.read(sensorValues);

  sensorValues[0] = map(sensorValues[0], 600, 1200, 0, 1000);
  sensorValues[1] = map(sensorValues[1], 800, 4095, 0, 1000);
  sensorValues[2] = map(sensorValues[2], 190, 4095, 0, 1000);
  sensorValues[3] = map(sensorValues[3], 177, 4095, 0, 1000);

  constrain(sensorValues[0], 0, 1000);
  constrain(sensorValues[1], 0, 1000);
  constrain(sensorValues[2], 0, 1000);
  constrain(sensorValues[3], 0, 1000);

}

void calcCrosses(){
  //sensorValues[0] = map(sensorValues[0], 0, 4, 0, 1000);

  //int sum = sensorValues[0] + sensorValues[1];
  if(sensorValues[0] + sensorValues[1] > 1500) crosses++, decisionFlag = 'L';
  else if(sensorValues[2] + sensorValues[3] > 1500) crosses++, decisionFlag = 'R';
  else crosses = 0, decisionFlag = 'N';

}

void statesEvolution(){
  tis = millis() - tes; //resetar tis

  if(state == start && tis > 2000){
    setState(1);

  }else if(state == front && crosses > 0){
    setState(decision);

  }else if(state == decision && decisionFlag == 'R'){
    setState(turnRight);

  }else if(state == decision && decisionFlag == 'L'){
    setState(turnLeft);

  }else if(state == turnRight && tis > 200){
    setState(front);

  }else if(state == turnLeft && tis > 200){
    setState(front);
    
  }else if(state == 99){
    
    
  }

}

void statesExits(){
  if(state == start){
    setRobotVW(0, 0);
  
  }else if(state == front){
    followLine();

  }else if(state == decision){
    setRobotVW(0, 0);

  }else if(state == turnRight){
    setRobotVW(0, 5);

  }else if(state == turnLeft){
    setRobotVW(0, -5);

  }else if(state == 99){
    followLine();

  }

}
