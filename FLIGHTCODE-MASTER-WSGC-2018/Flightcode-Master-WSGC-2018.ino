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
    //initialize I2C bus
    Wire.begin();
    //initialize serial bus
    Serial.begin(9600);
    delay(500);
    //perform I2C handshake procedure
    I2CHandshake();
}

void loop(){
    
}

bool I2CHandshake(){
    bool handshake;
    Serial.println("PERFORMING I2C HANDSHAKE TEST/n/r////Mega_MASTER->Nano_SLAVE/r/n");
    Wire.requestFrom(8, 10);
    while(Wire.available()){
      unsigned long read = Wire.read();
      Serial.println(read);
      if(read == HANDSHAKE_KEY){
        Serial.println("Key Found");
        handshake = true;
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
