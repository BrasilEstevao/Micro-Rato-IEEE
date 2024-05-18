#include <WiFi.h>

#include <Arduino.h>
#include "config.h"
#include "SparkFun_TB6612.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <QTRSensors.h>
#include "ezButton.h"

void TaskReadSensors(void *pvParameters);
void TaskStateManagement(void *pvParameters);
void TaskFollowLine(void *pvParameters);
void TaskSerialPrint(void *pvParameters);

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
float Ki = 0.000;
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

// Global variables for tasks
volatile int motorspeed = 0;
volatile int crosses = 0;
volatile char decisionFlag = 'N';
volatile int lastLeftSensors = 0;
volatile int lastRightSensors = 0;
volatile int nowSensors = 0;

volatile int irs_edge_sum = 0;
volatile int irs_edge_dif = 0;
volatile int irs_center_sum = 0;
volatile int irs_right_sum = 0;
volatile int irs_left_sum = 0;

//=========================================
//Function Headers

void followLine();
void setState(byte new_state);
void robot_forward();
void setRobotVW(float V, float W);
void readSensors();
void calcCrosses();
// void decideNextInstruction();
void callStatesTransitions();
void callStatesActions();

//=========================================

ezButton button(13);

void setup(){

  WiFi.mode(WIFI_OFF);
  btStop();

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

  // Create tasks
  xTaskCreate(TaskReadSensors, "ReadSensors", 2048, NULL, 1, NULL);
  xTaskCreate(TaskStateManagement, "StateManagement", 4096, NULL, 1, NULL);
  xTaskCreate(TaskFollowLine, "PID", 2048, NULL, 1, NULL);
  xTaskCreate(TaskSerialPrint, "SerialPrint", 2048, NULL, 1, NULL);

  // Start the scheduler
  vTaskStartScheduler();

  //delay(2000);

}

void loop()
{
}


void TaskReadSensors(void *pvParameters) {
    for (;;) {
        qtr.read(sensorValues);
        uint16_t position = qtr.readLineBlack(sensorValues);
        // nowSensors = sensorValues[1] + sensorValues[2];
        irs_edge_sum = sensorValues[0] + sensorValues[3];
        irs_edge_dif = sensorValues[0] - sensorValues[3];
        irs_center_sum = sensorValues[1] + sensorValues[2];
        irs_right_sum = sensorValues[0] + sensorValues[1];
        irs_left_sum = sensorValues[2] + sensorValues[3];
        // calcCrosses();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void TaskStateManagement(void *pvParameters) {
    for (;;) {
        callStatesTransitions();
        callStatesActions();

        // Check button press and set state accordingly
        button.loop();
        if(button.isPressed()) setState(start);

        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}

void TaskFollowLine(void *pvParameters) {
    for (;;) {
        if (state == follow) {
            followLine();
            int left_motorspeed = leftBaseSpeed + motorspeed;
            int right_motorspeed = rightBaseSpeed - motorspeed;
            constrain(right_motorspeed, 0, maxspeed);
            constrain(left_motorspeed, 0, maxspeed);
            right_motor.drive(right_motorspeed);
            left_motor.drive(left_motorspeed);
        }
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }
}

void followLine() {
    uint16_t position = qtr.readLineBlack(sensorValues);
    float error = 1500 - position;

    P = error;
    I += error;
    D = error - lastError;
    lastError = error;
    motorspeed = P * Kp + I * Ki + D * Kd;
}

void setState(byte new_state) {
    tes = millis();
    tis = 0;
    state = new_state;
}


/*

void readSensors() {
    qtr.read(sensorValues);

    sensorValues[0] = map(sensorValues[0], 600, 1200, 0, 1000);
    sensorValues[1] = map(sensorValues[1], 800, 4095, 0, 1000);
    sensorValues[2] = map(sensorValues[2], 190, 4095, 0, 1000);
    sensorValues[3] = map(sensorValues[3], 177, 4095, 0, 1000);

    constrain(sensorValues[0], 0, 1000);
    constrain(sensorValues[1], 0, 1000);
    constrain(sensorValues[2], 0, 1000);
    constrain(sensorValues[3], 0, 1000);
}*/

// Task for serial printing
void TaskSerialPrint(void *pvParameters) {
    for (;;) {
        Serial.print(" State: ");
        Serial.print(state);
        Serial.printf("IRSensors[E: %d  ED: %d  C: %d  R: %d  L: %d ]\n",irs_edge_sum,irs_edge_dif,irs_center_sum,irs_right_sum,irs_left_sum);
        vTaskDelay(50 / portTICK_PERIOD_MS);  // Adjust the delay as needed
    }
}

//===================================
//Funções




void robot_forward(){

  right_motor.drive(rightBaseSpeed);
  left_motor.drive(leftBaseSpeed); 

}

void setRobotVW(float V, float W){
  int leftDuty = map(V - W, 0, 100, 0, 255);
  int rightDuty = map(V + W, 0, 100, 0, 255);

  left_motor.drive(leftDuty);
  right_motor.drive(rightDuty);

}

// void readSensors(){
//   qtr.read(sensorValues);

//   sensorValues[0] = map(sensorValues[0], 600, 1200, 0, 1000);
//   sensorValues[1] = map(sensorValues[1], 800, 4095, 0, 1000);
//   sensorValues[2] = map(sensorValues[2], 190, 4095, 0, 1000);
//   sensorValues[3] = map(sensorValues[3], 177, 4095, 0, 1000);

//   constrain(sensorValues[0], 0, 1000);
//   constrain(sensorValues[1], 0, 1000);
//   constrain(sensorValues[2], 0, 1000);
//   constrain(sensorValues[3], 0, 1000);

// }

// void calcCrosses(){
//   //sensorValues[0] = map(sensorValues[0], 0, 4, 0, 1000);

//   //int sum = sensorValues[0] + sensorValues[1];
//   if(sensorValues[0] + sensorValues[1] > 1500) crosses++, lastRightSensors = sensorValues[0] + sensorValues[1];
//   else if(sensorValues[2] + sensorValues[3] > 1500) crosses++, lastLeftSensors = sensorValues[2] + sensorValues[3];
//   //else if(sensorValues[0] + sensorValues[1] + sensorValues[2] + sensorValues[3] > 3000) lastRightSensors = sensorValues[0] + sensorValues[1], lastLeftSensors = sensorValues[2] + sensorValues[3];
//   else crosses = 0, lastLeftSensors = 0, lastRightSensors = 0;

// }

// void decideNextInstruction(){
//   nowSensors = sensorValues[1] + sensorValues[2];

//   //Casos sem linha depois do entrocamento
//   if(nowSensors < 800 && lastLeftSensors < 1500 && lastRightSensors > 1500) decisionFlag = 'R'; //Curva simples pra direita
//   else if(nowSensors < 800 && lastLeftSensors > 1500 && lastRightSensors < 1500) decisionFlag = 'L'; //Curva simples pra esquerda
//   else if (nowSensors < 800 && lastLeftSensors > 1500 && lastRightSensors > 1500) decisionFlag = 'R'; //Entrocamento, direita é prioridade

//   //Casos com linha depois do entrocamento
//   else if(nowSensors > 800 && lastLeftSensors > 1500 && lastRightSensors < 1500) decisionFlag = 'F'; //Entrocamento com esquerda, seguir reto
//   else if(nowSensors > 800 && lastLeftSensors < 1500 && lastRightSensors > 1500) decisionFlag = 'R'; //Entrocamento com direita, virar a direita
//   else if(nowSensors > 800 && lastLeftSensors > 1500 && lastRightSensors > 1500) decisionFlag = 'R'; //Cruzamento, virar a direita


// }

void callStatesTransitions(){
  tis = millis() - tes; //resetar tis

  switch(state) {
    case States::start:
      if(tis > 2000) setState(States::follow);
    break;

    case States::follow:
      if(irs_right_sum > 1500 || irs_left_sum > 1500) setState(States::small_forward_1);
    break;

    case States::small_forward_1:
      if(irs_edge_dif > 500 && irs_center_sum > 1500)       setState(States::turn_right);
      else if(irs_edge_sum < 500 && irs_center_sum > 1500)  setState(States::follow);
      else if(irs_edge_sum < 500 && irs_center_sum < 500)   setState(States::turn_left);
      else if(irs_edge_sum > 1500 && irs_center_sum > 1500) setState(States::small_forward_2);
    break;

    case States::small_forward_2:
      if(irs_edge_sum < 500) setState(States::turn_right);
      else if(tis > 1000) setState(States::end);
    break;

    case States::turn_right:
      if(tis > 400) setState(States::follow);
    break;

    case States::turn_left:
      if(tis > 400) setState(States::follow);
    break;

    case States::turn_back:
      if(tis > 500) setState(States::follow);
    break;

    case States::end:
    break;
  }

  // if(state == start && tis > 2000){
  //   setState(follow);

  // }else if(state == follow && crosses > 0){
  //   setState(decision);

  // }else if(state == small_forward_1 && tis > 50){
  //   setState(rev);

  // }else if(state == rev && tis > 30)
  // {
  //   setState(decision);

  // }else if(state == decision && decisionFlag == 'L'){
  //   setState(turn_left);
  //   //setState(99);
  //   lastLeftSensors = 0;
  //   lastRightSensors = 0;

  // }else if(state == decision && decisionFlag == 'R'){
  //   setState(turn_right);
  //   //setState(99);
  //   lastLeftSensors = 0;
  //   lastRightSensors = 0;

  // }else if(state == decision && decisionFlag == 'F'){
  //   setState(follow);
  //   //setState(99);
  //   lastLeftSensors = 0;
  //   lastRightSensors = 0;
    
  // }else if(state == decision && decisionFlag == 'T'){
  //   setState(turn_back);
  //   //setState(99);
  //   lastLeftSensors = 0;
  //   lastRightSensors = 0;
    
  // }else if(state == turn_back && tis > 500){
  //   setState(follow);
    
  // }else if(state == turn_right && tis > 400){
  //   setState(follow);
    
  // }else if(state == turn_left && tis > 400){
  //   setState(follow);
    
  // }else if(state == 99){
    
    
  // }

}

void callStatesActions(){
  
  switch(state) {
    case States::start:
      setRobotVW(0,0);
    break;
    //case follow:
    //  followLine();
    //break;
    case States::small_forward_1:
      setRobotVW(20,0);
    break;
    // case rev:
    //   setRobotVW(-20,0);
    // break;
    // case decision:
    //   decideNextInstruction();
    // break;
    case States::turn_left:
      setRobotVW(0,20);
    break;
    case States::turn_right:
      setRobotVW(0,-20);
    break;
    case States::turn_back:
      setRobotVW(0,25);
    case States::end:
      setRobotVW(0,0);
    break;
  }
}
/*
void Rotate(void *pvParameter) {

  char *direction = (char *)pvParameter;  // Cast the parameter to a string (char pointer)

  unsigned long startTime = millis();

  if (strcmp(direction, "right") == 0) {
      // Start turning right
      setState(turn_right);
  } 
  else if (strcmp(direction, "left") == 0) {
      // Start turning left
      turn_left();
  }   
  else if (strcmp(direction, "turn back") == 0) {
      // do a 180
      turn_left();
      vTaskDelay(500);
  }
  else {
      // Invalid command 
      Serial.println("Invalid command");
      vTaskDelete(NULL);
      return;
  }

  vTaskDelay(200);
  
  // Wait for the specified duration or until the condition is met
 /* while ((millis() - startTime) < durationMs) {
    // Check if the condition to stop early is met
    if (checkCondition()) {
        break;
    }
    // Yield to allow other tasks to run
    vTaskDelay(1);  // Delay for 1 ms to yield control to other tasks
  }*/
  /*
  // Stop the motors
  stop_motors();

  // Delete the task
  vTaskDelete(NULL);

}
*/