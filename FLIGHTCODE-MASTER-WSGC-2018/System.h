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
int NANO_RST = 31;
int UV_SENSOR = 8;

//PREFLIGHT CHECKS
int BLUE_LED = 38;
int WHITE_LED = 39;
int RED_LED = 40;
int YELLOW_LED = 41;
int GREEN_LED = 42;
int PIEZO = 2;

//SPACEFIRE
Adafruit_AMG88xx amg;
Servo servo;

float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
int SpyCamera = 52;
int altitude;
int servoPosition = 0;
int servoincrement = 40;
int pos = 0; 
int SERVO_CTRL = 22;

int NICHROME_RELAY_1 = 23;
int NICHROME_RELAY_2 = 24;
int NICHROME_RELAY_3 = 26;
int NICHROME_RELAY_4 = 25;

//I2C ADDRESSES
int NANO = 8;

#endif
