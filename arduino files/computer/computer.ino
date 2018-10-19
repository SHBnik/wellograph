
// include external libs for nrf24l01+ radio transceiver communications
#include "RF24.h"
#include <SPI.h>
#include "nRF24l01.h"

// set Chip-Enable (CE) and Chip-Select-Not (CSN) radio setup pins
#define CE_PIN 9
#define CSN_PIN 10

// set transmission cycle send rate - in milliseconds
#define SEND_RATE 0

// create RF24 radio object using selected CE and CSN pins
RF24 radio(CE_PIN,CSN_PIN);

// setup radio pipe addresses for each sensor node
const byte nodeAddresses[5] = {'N','O','D','E','1'};

// integer to store count of successful transmissions
int masterDataSend[4];// [0] -> velocity  [1] -> tetha  [2} -> yaw

// simple integer array for data from each slave node: { node_id, returned_count }
int remoteNodeData[2] = {1,1};

// system operation timing variables - set SEND_RATE to limit transmit rate
unsigned long currentTime;
unsigned long lastSentTime;

/* Function: setup
 *    Initialises the system wide configuration and settings prior to start
 */
void setup()
{
    Serial.begin(9600);
    
    
    // begin radio object
    radio.begin();
    
    // set power level of the radio
    radio.setPALevel(RF24_PA_MAX);
    
    // set RF datarate - lowest rate for longest range capability
    radio.setDataRate(RF24_250KBPS);
    
    // set radio channel to use - ensure all slaves match this
    radio.setChannel(0x76);
    
    // set time between retries and max no. of retries
    radio.setRetries(4, 10);
    
    // enable ack payload - each slave replies with sensor data using this feature
    radio.enableAckPayload();

    
    Serial.println(20);
    
}




void loop()
{

    currentTime = millis();
    
//    if(currentTime - lastSentTime > SEND_RATE){
//      // collect sensor data from all nodes
//      receiveNodeData();
//      
//      lastSentTime = millis();
//    }


    if(Serial.available()){
      
      masterDataSend[0] = Serial.readStringUntil(',').toInt();
      Serial.read();
      masterDataSend[1] = Serial.readStringUntil(',').toInt();
      Serial.read();
      masterDataSend[2] = Serial.readStringUntil(',').toInt();
      Serial.read();
      masterDataSend[3] = Serial.readStringUntil('\n').toInt();
      receiveNodeData();
    }
    
}


void receiveNodeData()
{
     
     radio.openWritingPipe(nodeAddresses);
        
        bool tx_sent;
        
        tx_sent = radio.write( &masterDataSend, sizeof(masterDataSend) );
        
        if (tx_sent) {
            if (radio.isAckPayloadAvailable()) {                
                radio.read(&remoteNodeData, sizeof(remoteNodeData));               
            }
        }
        
}
