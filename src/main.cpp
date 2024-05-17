#include <Arduino.h>
#include "config.h"
#include "SparkFun_TB6612.h"
#include <QTRSensors.h>
#include "ezButton.h"

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
void decideNextInstruction();
void statesEvolution();
void statesExits();

//=========================================

int motorspeed = 0;
int crosses = 0;
char decisionFlag = 'N';
int lastLeftSensors = 0;
int lastRightSensors = 0;
int nowSensors = 0;

ezButton button(13);

void setup(){

button.setDebounceTime(10);

setState(100);
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

  button.loop();

  if(button.isPressed()) setState(start);

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
  
  // for (uint8_t i = 0; i < SensorCount; i++) 
  // {
  //   Serial.print(sensorValues[i]);
  //   Serial.print('\t');
  // }
  //Serial.println();
  // Serial.print(position);
  // Serial.print('\t');
  Serial.print("Now Sensors: ");
  Serial.print(nowSensors);
  
  Serial.print(" Last Left Sensors: ");
  Serial.print(lastLeftSensors);

  Serial.print(" Last Right Sensors: ");
  Serial.print(lastRightSensors);

  Serial.print(" Decision Flag: ");
  Serial.print(decisionFlag);

  Serial.print(" State: ");
  Serial.println(state);
  

  //Serial.print("Crosses: ");
  //Serial.println(crosses);

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
  if(sensorValues[0] + sensorValues[1] > 1500) crosses++, lastRightSensors = sensorValues[0] + sensorValues[1];
  else if(sensorValues[2] + sensorValues[3] > 1500) crosses++, lastLeftSensors = sensorValues[2] + sensorValues[3];
  //else if(sensorValues[0] + sensorValues[1] + sensorValues[2] + sensorValues[3] > 3000) lastRightSensors = sensorValues[0] + sensorValues[1], lastLeftSensors = sensorValues[2] + sensorValues[3];
  else crosses = 0, lastLeftSensors = 0, lastRightSensors = 0;

}

void decideNextInstruction(){
  nowSensors = sensorValues[1] + sensorValues[2];

  //Casos sem linha depois do entrocamento
  if(nowSensors < 800 && lastLeftSensors < 1500 && lastRightSensors > 1500) decisionFlag = 'R'; //Curva simples pra direita
  else if(nowSensors < 800 && lastLeftSensors > 1500 && lastRightSensors < 1500) decisionFlag = 'L'; //Curva simples pra esquerda
  else if (nowSensors < 800 && lastLeftSensors > 1500 && lastRightSensors > 1500) decisionFlag = 'R'; //Entrocamento, direita é prioridade

  //Casos com linha depois do entrocamento
  else if(nowSensors > 800 && lastLeftSensors > 1500 && lastRightSensors < 1500) decisionFlag = 'F'; //Entrocamento com esquerda, seguir reto
  else if(nowSensors > 800 && lastLeftSensors < 1500 && lastRightSensors > 1500) decisionFlag = 'R'; //Entrocamento com direita, virar a direita
  else if(nowSensors > 800 && lastLeftSensors > 1500 && lastRightSensors > 1500) decisionFlag = 'R'; //Cruzamento, virar a direita


}

void statesEvolution(){
  tis = millis() - tes; //resetar tis

  if(state == start && tis > 2000){
    setState(1);

  }else if(state == follow && crosses > 0){
    setState(buffer);

  }else if(state == buffer && tis > 50){
    // setState(decision);
    setState(decision);

  }else if(state == decision && decisionFlag == 'L'){
    setState(turnLeft);
    //setState(99);
    lastLeftSensors = 0;
    lastRightSensors = 0;

  }else if(state == decision && decisionFlag == 'R'){
    setState(turnRight);
    //setState(99);
    lastLeftSensors = 0;
    lastRightSensors = 0;

  }else if(state == decision && decisionFlag == 'F'){
    setState(follow);
    //setState(99);
    lastLeftSensors = 0;
    lastRightSensors = 0;
    
  }else if(state == decision && decisionFlag == 'T'){
    setState(turnBack);
    //setState(99);
    lastLeftSensors = 0;
    lastRightSensors = 0;
    
  }else if(state == turnBack && tis > 500){
    setState(follow);
    
  }else if(state == turnRight && tis > 200){
    setState(follow);
    
  }else if(state == turnLeft && tis > 200){
    setState(follow);
    
  }else if(state == 99){
    
    
  }

}

void statesExits(){
  if(state == start){
    setRobotVW(0, 0);
  
  }else if(state == follow){
    followLine();

  }else if(state == buffer){
    setRobotVW(2, 0);

  }else if(state == decision){
    decideNextInstruction();

  }else if(state == turnLeft){
    setRobotVW(0, 2);

  }else if(state == turnRight){
    setRobotVW(0, -2);

  }else if(state == turnBack){
    setRobotVW(0, 5);

  }else if(state == 99){
    setRobotVW(0, 0);

  }

}
