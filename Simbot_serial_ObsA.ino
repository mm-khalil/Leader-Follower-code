// Recieving serial data for obstacle avoidance to esp32
// This bot is not sending data to anyone

#include "HardwareSerial.h"
#include <Arduino_JSON.h>
//#include <TaskScheduler.h>

int inputValue = 0;
String inputString = "";
boolean stringComplete = false;
int SIMBotNumber = 1;  // A
String readings;
bool F, Re, R, L;

//------------- SIMBOT ------------------------
// PWM Pins for
int R_PWM = 10;
int L_PWM = 9;

//  Motor Direction Pins
int R_direction = 5;
int L_direction = 6;
#define Forward 1
#define Reverse 0

void clearbuffer ()
{ stringComplete = false;
  inputString = "";
}

void serialEvent()
{
  while (Serial.available())
  {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:

    if (inChar == '\n')
    {
      stringComplete = true;
      //Serial.println(inputString);
      //inputString= "";
    }
  }
}

void setup() {
  
  // put your setup code here, to run once:
  Serial.begin(57600);
  delay (300);

}

void loop() {
 
//      analogWrite(R_PWM, 100);
//      digitalWrite(R_direction, Forward);
//      analogWrite(L_PWM, 100);
//      digitalWrite(L_direction, Forward);

  // put your main code here, to run repeatedly:
  if (stringComplete)
  {
    // Do our coding here,,,
    JSONVar myObject = JSON.parse(inputString.c_str());
    F = myObject["F"];
    Re = myObject["Re"];
    R = myObject["R"];
    L = myObject["L"];


//   Serial.print(F ? "1" : "0");
//   Serial.print(Re ? "1" : "0");
//   Serial.print(R ? "1" : "0");
//   Serial.print(L ? "1" : "0");
//   Serial.println("");

    if (F == true &&  Re == false &&  R == false && L == false) {
      analogWrite(R_PWM, 100);
      digitalWrite(R_direction, Forward);
      analogWrite(L_PWM, 100);
      digitalWrite(L_direction, Forward);
    }

    else if (F == false &&  Re == true && R == false &&  L == false) {
      analogWrite(R_PWM, 40);
      digitalWrite(R_direction, Reverse);
      analogWrite(L_PWM, 40);
      digitalWrite(L_direction, Reverse);
    }

    else if (F == false &&  Re == false && R == true && L == false) {
      analogWrite(R_PWM, 40);
      digitalWrite(R_direction, Reverse);
      analogWrite(L_PWM, 180);
      digitalWrite(L_direction, Forward);
    }


    else if (F == false &&  Re == false && R == false && L == true) {
      analogWrite(R_PWM, 180);
      digitalWrite(R_direction, Forward);
      analogWrite(L_PWM, 40);
      digitalWrite(L_direction, Reverse);
    }
//    else
//    {
//      analogWrite(R_PWM, 255);
//      digitalWrite(R_direction, Forward);
//      analogWrite(L_PWM, 255);
//      digitalWrite(L_direction, Reverse);
//    }
    //   Serial.println(inputString);
    clearbuffer();
  }
  if (Serial.available() > 0)
  {
    serialEvent();
  }
}
