/**
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include "System.h"
#include <Wire.h>

void setup(){
    hardwareSetup();

   
    
    //initialize I2C bus
    Wire.begin();
    //initialize serial bus
    Serial.begin(9600);
    //perform I2C handshake procedure
    NanoI2CHandshake();
    Serial.println("Handshake Complete");
}

void loop(){
        digitalWrite(GREEN_LED, HIGH);
}

void hardwareSetup(){
    //Pin Config for Nano Reset
    pinMode(NANO_RST, OUTPUT);
    digitalWrite(NANO_RST, HIGH);

    //Pin Config for Pre-Flightcheck Board
    pinMode(GREEN_LED, OUTPUT);
    pinMode(2, OUTPUT);
}

//performs pre-flight check for I2C connection on the nano
bool NanoI2CHandshake(){
    bool handshake;
    //Console Feedback
    Serial.println("////PERFORMING I2C HANDSHAKE TEST////\n\r//////MASTER(Mega)->SLAVE(Nano)//////\r\n");
    //Reset Nano into test sequence and wait for startup
    Serial.println("Nano Wait");
    digitalWrite(NANO_RST, LOW);
    delay(1);
    digitalWrite(NANO_RST, HIGH);
    delay(2000);
    
    //Request One Byte
    Wire.requestFrom(8, 1);
    //Run until available
    while(Wire.available()){
      //get received value
      unsigned long read = Wire.read();
      Serial.println(read);

      //if keys match, break out of loop
      if(read == HANDSHAKE_KEY){
        Serial.println("Key Found");
        handshake = true;
        tone(2, 400, 500);
        digitalWrite(50, HIGH);
        break;
      }
      else{
        Serial.println("Key Not Found");
        handshake = false;
      }
      delay(100);
    }
    
    //echo over i2c to nano and listen for return
    //time delay? for handshake. true for complete false for no handshake
    
    return handshake;
}

void buzz(){
    tone(13, 250, 250);
}

