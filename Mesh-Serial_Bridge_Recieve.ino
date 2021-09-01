// ESP # 1
// Recieving ESP # 2 data from serial to wifi mesh
// sending data back to simbot 2

#include "HardwareSerial.h"
#include "painlessMesh.h"
#include <Arduino_JSON.h>

// MESH Details
#define   MESH_PREFIX     "RNTMESH" //name for your MESH
#define   MESH_PASSWORD   "MESHpassword" //password for your MESH
#define   MESH_PORT       5555 //default port


//Serial is used for debugging
// Serial2 is used for serial communication with other modules.
bool debug = false;
int inputValue = 0;
String inputString = "";
boolean stringComplete = false;

#define RXD2 16
#define TXD2 17


void taskserialCallback();
int SIMBotNumber = 2;
String readings;
void sendMessage ();

Scheduler userScheduler; // to control your personal task
Task taskserial(10000, TASK_FOREVER, &taskserialCallback); // 2sec, forever task
Task taskSendMessage(10000 , TASK_FOREVER, &sendMessage);  // 1 sec, forever task
painlessMesh  mesh;


void sendMessage () {
  //int i = 0;
}




void taskserialCallback() {
  //  String msg = getReadings();

  //Serial2.println(msg);                 // sending data back to the other module
  //  Serial.println(msg);

}

void clearbuffer ()
{ stringComplete = false;
  inputString = "";
}

void serialEvent()
{
  while (Serial2.available())
  {
    // get the new byte:
    char inChar = (char)Serial2.read();
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
// Needed for painless library
void receivedCallback( uint32_t from, String &msg ) {
  if (debug) {
    Serial.printf(msg.c_str());
  }
  Serial.println(msg);     // sending to matlab
 // Serial2.println(msg);  // Sending data back to simbot A...
}

void newConnectionCallback(uint32_t nodeId) {
  if (debug) {
    Serial.printf("New Connection, nodeId = %u\n", nodeId);
  }
}

void changedConnectionCallback() {
  if (debug) {
    Serial.printf("Changed connections\n");
  }

}

void nodeTimeAdjustedCallback(int32_t offset) {
  if (debug) {
    Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(), offset);
  }
}


void setup() {
  // Note the format for setting a serial port is as follows: Serial2.begin(baud-rate, protocol, RX pin, TX pin);
  Serial.begin(115200);
  Serial2.begin(57600, SERIAL_8N1, RXD2, TXD2);
  delay(1000);
  Serial.println("Serial Txd is on pin: " + String(TXD2));
  Serial.println("Serial Rxd is on pin: " + String(RXD2));

  mesh.setDebugMsgTypes( ERROR | STARTUP );  // set before init() so that you can see startup messages
  mesh.init( MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT );
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);


  userScheduler.addTask(taskSendMessage);
  taskSendMessage.enable();

  userScheduler.addTask(taskserial);
  taskserial.enable();
}

void loop()
{
  mesh.update();

  if (stringComplete)
  {
    mesh.sendBroadcast(inputString);
    if (debug) {
      Serial.println(inputString);
    }
    clearbuffer();
  }


  if (Serial2.available() > 0)
  {
    serialEvent();
  }

}
