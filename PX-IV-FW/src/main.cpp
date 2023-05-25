/*
* Prosthetic X PX-IV Firmware
* Version 1.0 created on 15-9-2021 by Léon Spek
* 
*/


#include <Arduino.h>

#ifdef ESP8266
 #include <ESP8266WiFi.h>
#else //ESP32
 #include <WiFi.h>
#endif
#include <ModbusIP_ESP8266.h>

#include <Servo.h>


#include "creds.h"
#include <ModeControl.h>
#include <WifiControl.h>

//#define POSTEST

#define PX_NUM 4


//PX units mostly need just 1 register for the 5 states they can be in
//registers start at 101 for PX-I
#define PX_REG 100 + (PX_NUM * 10) //base operational modbus register
#define PX_STATE_REG 200 + (PX_NUM * 10) // base status modbus register

 enum
  {
    PX_ERR = -1,
    PX_OK
  };

#define NUM_SERVOS 5

#define SERVO_MIN_POS 1471 // 1471//90
#ifdef POSTEST
#define SERVO_MAX_POS 1471 + ((2147 - 1471) / 2)
#else
#define SERVO_MAX_POS 2400//2200 //2147 
#endif
#define SERVO_STATE_DIFF 50




void setState(int8_t state);

//ModbusIP object
ModbusIP pxModbus;


char ssid[] = SSID ;        // your network SSID (name)
char pass[] = PW;                    // your network password
WifiControl pxWifi(ssid, pass, PX_NUM);


int8_t px4state = 0;

Servo pxServos[NUM_SERVOS];
int16_t servoGoals[NUM_SERVOS];
void detachAllServos();
void attachAllServos();

uint32_t dTime = 0;

bool demoMode = false;
uint32_t demoT = 0;
int8_t demoState = 0;

uint32_t wifiTime = 0;  //variable used to periodically look for our AP when initial connection fails


void demoCallback(uint32_t dTime, px_mode_t mode);
ModeControl pxMC(14, &demoCallback, 10000, &pxWifi);


void setup() {

  Serial.begin(115200);
  delay(1000);

  pxServos[0].attach(16, SERVO_MIN_POS, SERVO_MAX_POS); // D0
  pxServos[1].attach(5, SERVO_MIN_POS, SERVO_MAX_POS); // D1
  pxServos[2].attach(4, SERVO_MIN_POS, SERVO_MAX_POS); // D2
  pxServos[3].attach(0, SERVO_MIN_POS, SERVO_MAX_POS); // D3
  pxServos[4].attach(2, SERVO_MIN_POS, SERVO_MAX_POS); //D4

  for(uint8_t i = 0; i < NUM_SERVOS; i++){
    pxServos[i].writeMicroseconds(SERVO_MIN_POS);
    servoGoals[i] = SERVO_MIN_POS;
  }

  pxWifi.setPreConn(detachAllServos);
  pxWifi.setPostConn(attachAllServos);

  
  pxWifi.setTimeOut(30000);

  if(digitalRead(14) == HIGH){
  Serial.println("Connecting to C&C...");
  int8_t res = pxWifi.init();
  if(res == -1){
    Serial.println("No C&C found, starting up in demo mode!");
  }
  }

  //create the modbus server, and add a holding register (addHreg) and Ireg
  pxModbus.server(502);
  pxModbus.addHreg(PX_REG, 0);
  pxModbus.addIreg(PX_REG, 0);
  pxModbus.addHreg(PX_STATE_REG, PX_OK);

  pxMC.init(); //initialize modeControl
  Serial.println("setup finished.");


}

void loop() {

  

  //Call once inside loop() - all magic here
  pxModbus.task();


  //pxWifi.run();
  pxMC.run();

  
  //this copies the holding reg value to ireg
  if(pxModbus.Hreg(PX_REG) != pxModbus.Ireg(PX_REG)){
    pxModbus.Ireg(PX_REG, pxModbus.Hreg(PX_REG));
  }

  if(px4state != pxModbus.Ireg(PX_REG)){
    setState((int8_t)pxModbus.Ireg(PX_REG));
  }

  //attempt to reconnect to wifi in most neutral state
  //otherwise servos jerk / stop moving while reconnecting
  if(pxWifi.getStatus() != WL_CONNECTED && px4state == 0){
    int currPos = pxServos[0].readMicroseconds();

    if(abs(currPos - servoGoals[0]) < 10){
      pxWifi.reConn();
    }
  }
  if(millis() - dTime > 10){
    for(int8_t i = 0; i < NUM_SERVOS; i++)
    {
      int currPos = pxServos[i].readMicroseconds();
      pxServos[i].writeMicroseconds((servoGoals[i] * 0.025) + (currPos * 0.975));
    }

    dTime = millis();
  }


}

void setState(int8_t state){

  px4state = state;

  for(int8_t i = 0; i < NUM_SERVOS; i++){
    if(i < px4state){
      servoGoals[i] = SERVO_MAX_POS - (i * SERVO_STATE_DIFF) ;

    }else{
      servoGoals[i] = SERVO_MIN_POS;

    }
  }



}

void demoCallback(uint32_t dTime, px_mode_t mode){
  if(mode == PX_DEMO_MODE){
  if(demoState > 9){
        demoState = 0;
        
      }
      if(demoState > 5){
        pxModbus.Hreg(PX_REG, 10 - demoState);
        Serial.print("Checking PX status reg in demo mode: ");
        Serial.println(pxModbus.Hreg(PX_REG));
      }else{
        pxModbus.Hreg(PX_REG, demoState);
        Serial.print("Checking PX status reg in demo mode: ");
        Serial.println(pxModbus.Hreg(PX_REG));
      }
      demoState++;
  }else{
    demoState = 0;
    pxModbus.Hreg(PX_REG, demoState);
  }
}

void detachAllServos(){
  pxServos[0].detach(); // D0
    pxServos[1].detach();  // D1
    pxServos[2].detach();  // D2
    pxServos[3].detach();  // D3
    pxServos[4].detach();  //D4
}

void attachAllServos()
{
  pxServos[0].attach(16, SERVO_MIN_POS, SERVO_MAX_POS); // D0
    pxServos[1].attach(5, SERVO_MIN_POS, SERVO_MAX_POS);  // D1
    pxServos[2].attach(4, SERVO_MIN_POS, SERVO_MAX_POS);  // D2
    pxServos[3].attach(0, SERVO_MIN_POS, SERVO_MAX_POS);  // D3
    pxServos[4].attach(2, SERVO_MIN_POS, SERVO_MAX_POS);  //D4
}