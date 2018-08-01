/**
   Licensed to the Apache Software Foundation (ASF) under one
   or more contributor license agreements.  See the NOTICE file
   distributed with this work for additional information
   regarding copyright ownership.  The ASF licenses this file
   to you under the Apache License, Version 2.0 (the
   "License"); you may not use this file except in compliance
   with the License.  You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing,
   software distributed under the License is distributed on an
   "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
   KIND, either express or implied.  See the License for the
   specific language governing permissions and limitations
   under the License.
*/

#include "System.h"
#include <Wire.h>
#include <Adafruit_AMG88xx.h>
#include <Servo.h>



void setup() {
  Serial.begin(9600);
  Serial.println(">>>>>>WSGC HIGH-ALTITUDE BALLOON PAYLOAD 2018<<<<<<");
  Serial.println("Chapman-Adridge | Bock | Jackson | Nettesheim | Dregne | Buchmann\n\r");
  Serial.println(">>>>Performing System Checks<<<<");
  hardwareSetup();
  //initialize I2C bus
  Wire.begin();
  //initialize serial bus
  
  //perform I2C handshake procedure
  NanoI2CHandshake();
  Serial.println("> (1/5) Nano Handshake Complete");
}

void loop() {
  Wire.requestFrom(NANO, 4);
  Serial.print("Geiger: ");
  Serial.println(Wire.read());
  delay(500);
}

void hardwareSetup() {
  //Pin Config for Nano Reset
  pinMode(NANO_RST, OUTPUT);
  digitalWrite(NANO_RST, HIGH);

  //Pin Config for Pre-Flightcheck Board
  pinMode(GREEN_LED, OUTPUT);
  pinMode(2, OUTPUT);
}

void spaceFireSetup() {
  myservo.attach(SERVO_PIN);  // attaches the servo on pin 9 to the servo object
  pinMode(SpyCamera, OUTPUT);
  bool status;
  // default settings
  status = amg.begin();
  if (!status) {
    Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
    while (1);
  }
  delay(100); // let sensor boot up
}

void spaceFireRun() {
  //add back in when quin finishes it
  //altitude = getAltitude();
  if ((altitude > 2590 && altitude < 2650) || (altitude > 2743 && altitude > 2803) || (altitude > 2895 && altitude < 2955) || (altitude > 3050 && altitude < 3100)) {
    servoPosition = servoPosition + servoincrement;
    myservo.write(servoPosition);

    digitalWrite(SpyCamera, LOW);
    delay(555);
    digitalWrite(SpyCamera, HIGH);

  }
}

//performs pre-flight check for I2C connection on the nano
bool NanoI2CHandshake() {
  bool handshake;
  //Console Feedback
  Serial.println("////PERFORMING I2C HANDSHAKE TEST////\n\r//////MASTER(Mega)->SLAVE(Nano)//////\r\n");
  //Reset Nano into test sequence and wait for startup
  Serial.println("> Nano Wait (2000 ms)");
  digitalWrite(NANO_RST, LOW);
  delay(1);
  digitalWrite(NANO_RST, HIGH);
  delay(2000);

  //Request One Byte
  Wire.requestFrom(NANO, 1);
  //get received value
  unsigned long read = Wire.read();
  Serial.println("> KEY: "+ (String)read);

  //if keys match, break out of loop
  if (read == HANDSHAKE_KEY) {
    Serial.println("> Key Found");
    handshake = true;
    tone(PIEZO, 400, 500);
    digitalWrite(BLUE_LED, HIGH);
  }
  else {
    Serial.println("> Key Not Found");
    handshake = false;
    digitalWrite(RED_LED, HIGH);
  }
  delay(100);
  return handshake;
}


void getUVSensor() {
  int sensorValue = analogRead(7);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float voltage1 = sensorValue * (3.3 / 1023);
  float voltage = voltage1 / 0.1;
  Serial.println(voltage);
}



