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

#ifndef SYSTEM_H
#define SYSTEM_H

#include <Adafruit_AMG88xx.h>
#include <Servo.h>

//key used for I2C Mega->Nano preflight check
unsigned long HANDSHAKE_KEY = 0x3b;

//PINS
int NANO_RST = 53;

//PREFLIGHT CHECKS
int GREEN_LED = 52;
int RED_LED = 51;
int BLUE_LED = 50;
int WHITE_LED = 49;
int YELLOW_LED = 48;
int PIEZO = 2;

//SPACEFIRE
Adafruit_AMG88xx amg;
Servo myservo;

float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
int SpyCamera = 4;
int altitude;
int servoPosition = 35;
int servoincrement = 35;
int pos = 0; 
int SERVO_PIN = 999999;

//I2C ADDRESSES
int NANO = 8;

#endif
