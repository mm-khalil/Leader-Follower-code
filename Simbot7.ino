// Sending status to other robot ******* (sending serial data to esp32)
// This robot will not recieve anything from serial


#include "HardwareSerial.h"
#include <Arduino_JSON.h>
#include <TaskScheduler.h>
#include <SoftwareSerial.h>

int inputValue = 0;
String inputString = "";
boolean stringComplete = false;
int SIMBotNumber = 2;
String readings;

//------------- SIMBOT ------------------------
// PWM Pins for
int R_PWM = 10;
int L_PWM = 9;

//  Motor Direction Pins
int R_direction = 5;
int L_direction = 6;
#define Forward 1
#define Reverse 0

//  Motor Encoder Pins
int R_encoder = 2;
int L_encoder = 3;
long int encoderLPos = 0;
long int encoderRPos = 0;


SoftwareSerial mySerial(7,8);
  
int IR_enable = 4;
int OT = 900; // 0 white close obstacle -- 1023 no obstacle

// Variables for 5 IR proximity sensors
int OR, ORF, OF, OLF, OL;
bool L2, L1, F0, R1, R2;
bool front_obstacle = 0;
bool F, Re, R, L = 0;

float  dia = 0.034  ;         // wheel diameter (in mm)
float Dl, Dr, Dc, Ori_ch;
float Ar, Al, A;

float ER = 135;      //1m = 1350 rev
float x = 0;           // x initial coordinate of mobile robot
float y = 0;           // y initial coordinate of mobile robot
float Ori  = 0;       // The initial orientation of mobile robot
float Pi = 3.14;
float b = 0.084 ;     // b is the wheelbase of the mobile robot in mm

void t1Callback();
Task t1(100, TASK_FOREVER, &t1Callback);
Scheduler runner;

void t1Callback() {
  SendReadings(F, Re, R, L); // Send readings to other Simbot
}
/*String*/
void SendReadings (bool F, bool Re, bool R, bool L) {
  JSONVar jsonReadings;
  jsonReadings["F"] = F;
  jsonReadings["Re"] = Re;
  jsonReadings["R"] = R;
  jsonReadings["L"] = L;
  readings = JSON.stringify(jsonReadings);
  //return readings;

  Serial.println(readings);
  mySerial.println(readings);
}

void setup() {

  Serial.begin (57600);
  mySerial.begin(57600);
  // Taking Inpute from Encoders
  pinMode(R_encoder, INPUT);
  pinMode(L_encoder, INPUT);
  pinMode(IR_enable, OUTPUT);
  // Counting revolutions of right and left wheel
  attachInterrupt(digitalPinToInterrupt(R_encoder), do_REncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(L_encoder), do_LEncoder, CHANGE);

  runner.addTask(t1);
  t1.enable();
}

void loop() {
   runner.execute();
   IR_proximity_read();
   check_obstacle();
   avoid_and_move();
  // Send_sensor_readings();
  // delay(100);        // delay in between reads for stability
  //right();

}

void check_obstacle() {

  if (OL < OT) {
    L2 = 1;
  }
  else {
    L2 = 0;
  }

  if (OLF < OT) {
    L1 = 1;
  }
  else {
    L1 = 0;
  }



  if (OF < OT) {
    F0 = 1;
  }
  else {
    F0 = 0;
  }

  if (ORF < OT) {
    R1 = 1;
  }
  else {
    R1 = 0;
  }

  if (ORF < OT) {
    R1 = 1;
  }
  else {
    R1 = 0;
  }

  if (OR < OT) {
    R2 = 1;
  }
  else {
    R2 = 0;
  }
}

void Go() {
  F = 1;
  Re = 0;
  R = 0;
  L = 0;


  analogWrite(R_PWM, 100);
  digitalWrite(R_direction, Forward);
  analogWrite(L_PWM, 100);
  digitalWrite(L_direction, Forward);

}

void retreat() {
  F = 0;
  Re = 1;
  R = 0;
  L = 0;

  analogWrite(R_PWM, 40);
  digitalWrite(R_direction, Reverse);
  analogWrite(L_PWM, 40);
  digitalWrite(L_direction, Reverse);
}

void right() {
  F = 0;
  Re = 0;
  R = 1;
  L = 0;

  analogWrite(R_PWM, 40);
  digitalWrite(R_direction, Reverse);
  analogWrite(L_PWM, 180);
  digitalWrite(L_direction, Forward);
}

void left() {
  F = 0;
  Re = 0;
  R = 0;
  L = 1;
   
  analogWrite(R_PWM, 180);
  digitalWrite(R_direction, Forward);
  analogWrite(L_PWM, 40);
  digitalWrite(L_direction, Reverse);
}
void Stop() {
  //  Serial.println("Stop");
  //        analogWrite(R_PWM, 255);
  //      analogWrite(L_PWM, 255);
  // delay(1000);
}

void IR_proximity_read() {   // read IR sensors
  int n = 5; // average parameter
  digitalWrite(IR_enable, HIGH);  //IR Enable
  OR = 0;
  ORF = 0;
  OF = 0;
  OLF = 0;
  OL = 0;
  for (int i = 0; i < n; i++) {
    OL += analogRead(A3);
    OLF += analogRead(A2);
    OF += analogRead(A1);
    ORF += analogRead(A0);
    OR += analogRead(A7);
    delay(5);
  }
  OR /= n;
  ORF /= n;
  OF /= n;
  OLF /= n;
  OL /= n;
}
void Send_sensor_readings() {
  // Serial.print(IR_right);
  // Serial.print(',');
  // Serial.print(IR_right_front);
  // Serial.print(',');
  //Serial.println(OF);
  // Serial.print(',');
  // Serial.print(IR_left_front);
  // Serial.print(',');
  // Serial.println(IR_left);
}

void avoid_and_move() {
  if (L2 == 0 && L1 == 0 && F0 == 0 && R1 == 0 && R2 == 0) {
    Go();

  }
  else if (L2 == 0 && L1 == 0 && F0 == 1 && R1 == 0 && R2 == 0) {
    Stop();
    retreat();

  }

  else if (L2 == 0 && L1 == 1 && F0 == 0 && R1 == 0 && R2 == 0) {
    Stop();
    right();
  }

  else if (L2 == 0 && L1 == 1 && F0 == 1 && R1 == 0 && R2 == 0) {
    Stop();
    retreat();
    right();
  }
  else if (L2 == 1 && L1 == 0 && F0 == 1 && R1 == 0 && R2 == 0) {
    Stop();
    retreat();
    right();
  }
  else if (L2 == 1 && L1 == 1 && F0 == 1 && R1 == 0 && R2 == 0) {
    Stop();
    retreat();
    right();
  }
  else if (L2 == 1 && L1 == 0 && F0 == 0 && R1 == 0 && R2 == 0) {
    Stop();
    right();
  }
  else if (L2 == 1 && L1 == 1 && F0 == 0 && R1 == 0 && R2 == 0) {
    Stop();
    right();
  }

  else if (L2 == 0 && L1 == 0 && F0 == 0 && R1 == 1 && R2 == 0) {
    Stop();
    left();
  }

  else if (L2 == 0 && L1 == 0 && F0 == 1 && R1 == 1 && R2 == 0) {
    Stop();
    retreat();
    left();
  }
  else if (L2 == 0 && L1 == 0 && F0 == 1 && R1 == 0 && R2 == 1) {
    Stop();
    retreat();
    left();
  }
  else if (L2 == 0 && L1 == 0 && F0 == 1 && R1 == 1 && R2 == 1) {
    Stop();
    retreat();
    left();
  }
  else if (L2 == 0 && L1 == 0 && F0 == 0 && R1 == 0 && R2 == 1) {
    Stop();
    left();
  }
  else if (L2 == 0 && L1 == 0 && F0 == 0 && R1 == 1 && R2 == 1) {
    Stop();
    left();
  }
  else if (L2 == 0 && L1 == 1 && F0 == 1 && R1 == 1 && R2 == 0) {
    Stop();
    retreat();
  }
  else if (L2 == 1 && L1 == 1 && F0 == 1 && R1 == 1 && R2 == 1) {
    Stop();
    retreat();
  }
}

// Simbot Functions
void do_LEncoder() {
  if (F == 1)
  {
    encoderLPos++;
  }
  else if (Re == 1)
  {
    encoderLPos--;
  }
  else if (R == 1)
  {
    encoderLPos++;
  }
  else if (L == 1)
  {
    encoderLPos--;
  }
}
void do_REncoder() {
  if (F == 1)
  {
    encoderRPos++;
  }
  else if (Re == 1)
  {
    encoderRPos--;
  }
  else if (R == 1)
  {
    encoderRPos--;
  }
  else if (L == 1)
  {
    encoderRPos++;
  }
}

void SetEncoder() {
  encoderLPos = 0;
  encoderRPos = 0;

}
