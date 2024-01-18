#include "WiFi.h"
#include "AsyncUDP.h"
#include <Wire.h>                        // This is for i2C



const char * ssid = "***********";
const char * password = "***********";
int loopCounter = 0;
int rotations = 0;
int lastAngle = 0;

// ----- AS5600 Magnetic sensor
int magnetStatus = 0;                                   //value of the status register (MD, ML, MH)
int lowbyte;                                            //raw angle bits[7:0]
word highbyte;                                          //raw angle bits[11:8]
int rawAngle;                                           //final raw angle bits[11:0]
float degAngle;                                         //raw angle in degrees (360/4096 * [value between 0-4095])


int toggle = HIGH;
uint16_t helm_required;
String helm_string;
AsyncUDP udp;

void setup(){
    uint8_t* bufferpt; 
   
    pinMode(LED_BUILTIN, OUTPUT);    
    Serial.begin(115200);
    
    Wire.begin();                                         // start i2C
    Wire.setClock(400000L);                               // fast clock 

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    digitalWrite(LED_BUILTIN, LOW);
    delay(4000);     
    while (WiFi.waitForConnectResult() != WL_CONNECTED) {
        digitalWrite(LED_BUILTIN, HIGH);      
        Serial.println("WiFi Failed");
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100); 
        digitalWrite(LED_BUILTIN, LOW);    
        delay(900);   
        WiFi.begin(ssid, password);
        
    }
    Serial.print("Local IP: ");    
    Serial.println(WiFi.localIP());
    delay(3000);

    checkMagnetPresence();                              // check the magnet (blocks until magnet is found)
  
    if(udp.listen(8005)) {
        Serial.print("UDP Listening on IP: ");
        Serial.println(WiFi.localIP());
        udp.onPacket([](AsyncUDPPacket packet) {
            uint8_t buf[20];
            size_t l = packet.read(buf, 20);                        
            Serial.print("Data: ");
            Serial.write(packet.data(), packet.length());
            Serial.println();                            
            if (buf[0] == 1u && buf[3] == 1u && packet.length() == 4) {            
                helm_required = ((unsigned)buf[1] << 8) | buf[2];                    
                
                //reply to the client
                packet.printf("Got cmd %u  helm required %u\n", buf[0], helm_required);
            }else{
                packet.printf("Got %u bytes of data", packet.length());               
            }
        });
    }
}

void loop(){
    delay(100);
    ++loopCounter;
    //Send broadcast
    if (loopCounter == 50){
        loopCounter = 0;
        toggle ^= HIGH;
        digitalWrite(LED_BUILTIN, toggle);
        helm_string = String(helm_required) + " R:" + String(rotations) + " A:" + String(rawAngle);
        udp.broadcast(helm_string.c_str());
        Serial.printf("Helm required: %u \n", helm_required);
    }
    readRawAngle();
    computeRotations();
    if (loopCounter & 4u == 4u){ 
        degAngle = rawAngle * 0.087890625;     // 360/4096 = 0.087890625
        Serial.printf("Rotations: %i Raw: %u Deg angle: %3.2f \n", rotations, rawAngle, degAngle);  
    }
    
}


// ----------------------
// checkMagnetPresence()
// ----------------------
void checkMagnetPresence(){
  //Status register output: 0 0 MD ML MH 0 0 0
  //MH: Too strong magnet  -  101000  - dec 40
  //ML: Too weak magnet -     110000  - dec 48
  //MD: OK magnet -           100000  - dec 32

  // ---- Check MD status bit (magnet detected)
  while ((magnetStatus & B00100000) != 32) {                   // locks MCU until magnet correctly positioned
    magnetStatus = 0; //reset reading

    Wire.beginTransmission(0x36);                               //connect to the sensor
    Wire.write(0x0B);                                           //figure 21 - register map: Status: MD ML MH
    Wire.endTransmission();                                     //end transmission
    Wire.requestFrom(0x36, 1);                                  //request from the sensor
    while (Wire.available() == 0);                              //wait until it becomes available
    magnetStatus = Wire.read() & B00111000;                     //Reading the data after the request
    Serial.print("Magnet Status  ");
    Serial.println(magnetStatus, BIN);
    Serial.println(" ");
  }
  Serial.println(magnetStatus, DEC);
  delay(1000);
}


// ----------------
// readRawAngle()
//
// ----------------
void readRawAngle(){
  //----- read low-order bits 7:0
  Wire.beginTransmission(0x36);                         //connect to the sensor
  Wire.write(0x0D);                                     //figure 21 - register map: Raw angle (7:0)
  Wire.endTransmission();                               //end transmission
  Wire.requestFrom(0x36, 1);                            //request from the sensor
  while (Wire.available() == 0);                        //wait until it becomes available
  lowbyte = Wire.read();                                //Reading the data after the request

  // ----- read high-order bits 11:8
  Wire.beginTransmission(0x36);
  Wire.write(0x0C);                                     //figure 21 - register map: Raw angle (11:8)
  Wire.endTransmission();
  Wire.requestFrom(0x36, 1);
  while (Wire.available() == 0);
  highbyte = Wire.read();

  // ----- combine bytes
  highbyte = highbyte << 8;                             // shift highbyte to left
  rawAngle = highbyte | lowbyte;                        // combine bytes to get 12-bit value 11:0
 
  //Serial.print("Deg angle: ");
  //Serial.println(degAngle, 2);                          //absolute position of the encoder within the 0-360 circle
}


// ----------------
// computeRotations()
// Counts the number of rotations clockwise positive
// ----------------
void computeRotations(){
    if (rawAngle < 1028 && lastAngle > 3084){
       ++rotations;
    } else if (rawAngle > 3084 && lastAngle < 1028){
      --rotations;

    }
    lastAngle = rawAngle;

}


