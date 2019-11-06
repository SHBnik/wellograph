
// nRF24L01 radio transceiver external libraries
#include <SPI.h>
#include "RF24.h"
#include "nRF24L01.h"
#include "printf.h"
#include "motors.h"

#include <Servo.h>

Servo pen;
// chip select and RF24 radio setup pins
#define CE_PIN 48
#define CSN_PIN 49
RF24 radio(CE_PIN,CSN_PIN);

// setup radio pipe addresses for each sensor node
const byte nodeAddress[5] = {'N','O','D','E','1'};

// simple integer array for remote node data, in the form [node_id, returned_count]
int remoteNodeData[2] = {1, 1};

int dataFromMaster[4]; // [0] -> velocity  [1] -> tetha  [2} -> yaw [3] -> dot


bool no_dot = true;




void create_dot(){
  motorA_move(0);
  motorB_move(0);
  motorC_move(0);
  pen.write(100);
  delay(300);
  pen.write(180);
  delay(300);
}



void setup() {
  pen.attach(13);
  pen.write(180);
  Serial.begin(9600);
 
  radio.begin();
  
  // set power level of the radio
  radio.setPALevel(RF24_PA_LOW);

  // set RF datarate
  radio.setDataRate(RF24_250KBPS);

  // set radio channel to use - ensure it matches the target host
  radio.setChannel(0x76);

  // open a reading pipe on the chosen address - matches the master tx
  radio.openReadingPipe(1, nodeAddress);     

  // enable ack payload - slave replies with data using this feature
  radio.enableAckPayload();

  // preload the payload with initial data - sent after an incoming message is read
  radio.writeAckPayload(1, &remoteNodeData, sizeof(remoteNodeData));

  // print radio config details to console
  printf_begin();
  radio.printDetails();
//  while(1);

  // start listening on radio
  radio.startListening();
  
  // --------------------------------------------------------------------------------------------//
}




void radioCheckAndReply(void);


void loop() {
  
  radioCheckAndReply();


  float _theta = dataFromMaster[1]/* tetha *//180.00*pi;
  int mag = dataFromMaster[0] /* velo */;
  
  float Vx = mag * cos(_theta);
  float Vy = mag * sin(_theta);

  float A = -Vx;
  float B = 0.5 * Vx - sqrt(3)/2 * Vy;
  float C = 0.5 * Vx + sqrt(3)/2 * Vy;



  A = constrain(A,-500,500);
  B = constrain(B,-500,500);
  C = constrain(C,-500,500);

  A_speed = map(A,-500,500,-255,255);
  B_speed = map(B,-500,500,-255,255);
  C_speed = map(C,-500,500,-255,255);

  motorA_move(constrain(A_speed + dataFromMaster[2],-255,255)/*yaw*/);
  motorB_move(constrain(B_speed + dataFromMaster[2],-255,255)/*yaw*/);
  motorC_move(constrain(C_speed + dataFromMaster[2],-255,255)/*yaw*/);

  if(dataFromMaster[3] == 1 && no_dot == false){
    create_dot();
    no_dot = true;
  }
}










void radioCheckAndReply(void)
{
    // check for radio message and send sensor data using auto-ack
    if ( radio.available() ) {
          radio.read( &dataFromMaster, sizeof(dataFromMaster) );
          Serial.print("data : ");
          Serial.print(dataFromMaster[0]);
          Serial.print(" , ");
          Serial.print(dataFromMaster[1]);
          Serial.print(" , ");
          Serial.println(dataFromMaster[2]);
          Serial.print(" , ");
          Serial.println(dataFromMaster[3]);
          no_dot = false;
          radio.writeAckPayload(1, &remoteNodeData, sizeof(remoteNodeData));
    }
}
