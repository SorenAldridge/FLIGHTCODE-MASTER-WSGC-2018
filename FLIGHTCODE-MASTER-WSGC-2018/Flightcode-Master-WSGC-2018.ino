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
#include <SPI.h>
#include "SdFat.h"
#include "RTClib.h"
#include "Adafruit_BME680.h"
#include "Adafruit_SGP30.h"

RTC_PCF8523 RTC;  

void setup() {
  Serial.begin(9600);
  Serial.println(">>>>>>WSGC HIGH-ALTITUDE BALLOON PAYLOAD 2018<<<<<<");
  Serial.println("Chapman-Adridge | Bock | Jackson | Nettesheim | Dregne | Buchmann\n\r");
  Serial.println(">>>>Performing System Checks<<<<");
  hardwareSetup();
  //initialize I2C bus
  Wire.begin(); 
  //perform I2C handshake procedure
  NanoI2CHandshake();
  Serial.println("> (1/5) Nano Handshake Complete");
  atmosphericSetup();
  spaceFireSetup();

  //SYNC RTC CLOCK
  RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
}

void hardwareSetup() {
  //Pin Config for Nano Reset
  pinMode(NANO_RST, OUTPUT);
  digitalWrite(NANO_RST, HIGH);
}

void spaceFireSetup() {
  myservo.attach(SERVO_CTRL);  // attaches the servo on pin 9 to the servo object
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

void runSpaceFire() {
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
    tone(PIEZO, 222, 500);
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

#define SEALEVELPRESSURE_HPA (1013.25)  // change this number based upon the forcast that day
#define LOG_INTERVAL  1000              // mills between entries
#define SYNC_INTERVAL 1000              // mills between calls to flush() - to write data to the card
#define ECHO_TO_SERIAL   1              // echo data to serial port
#define FILE_BASE_NAME "Data"           // Log file base name.  Must be six characters or less.

                       // define the Real Time Clock object

uint32_t syncTime = 0;                  // time of last sync
const int chipSelect = 10;              // for the data logging shield, we use digital pin 10 for the SD cs line
uint32_t logTime;                       // Time in micros for next data record.

SdFat sd;             // File system object.
SdFile logfile;       // Log file.
Adafruit_BME680 bme;  // I2C Object for BME680 Sensor
Adafruit_SGP30 sgp;   // I2C Object for SGP30 Sensor

//*************************** Define Analog Pins ****************************
int CFC_Pin = 4;
int Benzene_Pin = 5;
int Ozone_Gas_Pin = 0;
int Ozone_Ref_Pin = 1;
int Ozone_Temp_Pin = 2;
int Oxygen_Pin = 3;


void logString(String toLog){
  logfile.print(toLog);
  logfile.println();
}

String getAtmosphericData(){
  // DateTime object
  DateTime now = RTC.now();
  
  // Log milliseconds since starting
  uint32_t m = millis(); 

  // Read all sensors 
  float temp = bme.temperature;
  float pressure = bme.pressure;
  float humidity = bme.humidity;
  float gas_resistance = bme.gas_resistance / 1000.0;
  float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  float TVOC = sgp.TVOC;
  float eCO2 = sgp.eCO2;
  float CFC_reading = analogRead(CFC_Pin);
  float Benzene_reading = analogRead(Benzene_Pin);
  float ozoneGas = analogRead(Ozone_Gas_Pin);
  float ozoneRef = analogRead(Ozone_Ref_Pin);
  float ozoneTemp = analogRead(Ozone_Temp_Pin);
  float oxygenReading = analogRead(Oxygen_Pin);
  

  // Guess what BA means
  String BAString = String(m) + ", " + String(now.unixtime()) + ", " + String(now.year()) + "/" + String(now.month()) + "/" + String(now.day()) + " " + 
                    String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second()) + ", "+ String(temp) + ", " + String(pressure) + ", " + 
                    String(humidity) + ", " + String(gas_resistance) + ", " + String(altitude) + ", " + String(TVOC) + ", " + String(eCO2) + ", " + 
                    String(CFC_reading) + ", " + String(Benzene_reading) + ", " + String(ozoneGas) + ", " + String(ozoneRef) + ", " + String(ozoneTemp) + ", " + String(oxygenReading);
 
  return BAString;
}

int getAltitude(){
  return bme.readAltitude(SEALEVELPRESSURE_HPA);
}

bool checkBME(){
  if (! bme.performReading())
    return false;
  else
    return true;
}

bool checkSGP(){
  if (! sgp.IAQmeasure())
    return false;
  else
    return true;
}

bool BMEstartup(){
  if (!bme.begin())
    return false;
  else 
    return true;
}

bool SGPstartup(){
  if (!sgp.begin()){
    return false;
  }
  else 
    return true;
}

bool RTCstartup(){
  Wire.begin();  
  if (!RTC.begin())
    return false;
  else{ 
    return true;
  }
}

bool fileSetup(){
  bool didWork = true;
  const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
  char fileName[13] = FILE_BASE_NAME "00.csv";
  
  // Initialize at the highest speed supported by the board that is not over 50 MHz. Try a lower speed if SPI errors occur.
  if (!sd.begin(chipSelect, SD_SCK_MHZ(50))) {
    sd.initErrorHalt();
  }

  // Find an unused file name.
  if (BASE_NAME_SIZE > 6) 
  {
    didWork = false;
  }
  while (sd.exists(fileName)) 
  {
    if (fileName[BASE_NAME_SIZE + 1] != '9') 
    {
      fileName[BASE_NAME_SIZE + 1]++;
    }
    else if (fileName[BASE_NAME_SIZE] != '9') 
    {
      fileName[BASE_NAME_SIZE + 1] = '0';
      fileName[BASE_NAME_SIZE]++;
    } 
    else 
    {
      didWork = false;
    }
  }
  if (!logfile.open(fileName, O_CREAT | O_WRITE | O_EXCL)) 
  {
    didWork = false;
  }
}

void initLogTime(){
  // Start on a multiple of the sample interval.
  logTime = micros()/(1000UL*LOG_INTERVAL) + 1;
  logTime *= 1000UL*LOG_INTERVAL;
}

void initBME(){
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
}


void initSGP(){
  Serial.begin(9600);
  Serial.println("SGP30 test");

  if (! sgp.begin()){
    Serial.println("Sensor not found :(");
    while (1);
  }
  Serial.print("Found SGP30 serial #");
  Serial.print(sgp.serialnumber[0], HEX);
  Serial.print(sgp.serialnumber[1], HEX);
  Serial.println(sgp.serialnumber[2], HEX);

  // If you have a baseline measurement from before you can assign it to start, to 'self-calibrate'
  //sgp.setIAQBaseline(0x8E68, 0x8F41);  // Will vary for each sensor!
}

void atmosphericSetup() {
  if(BMEstartup()){
    // Ready to go (Do something with LED)
  }
  if(SGPstartup()){
    // Ready to go (Do something with LED)
  }
  if(RTCstartup){
    // Ready to go (Do something with LED)
  }
  if(fileSetup()){
    // Ready to go (Do something with LED)
  }

  initLogTime();
  initBME();
  Serial.println("Millis,Stamp,Datetime,Temperature,Pressure,Humidity,Gas,Approx. Altitude,TVOC,eCO2,CFC,Benzene,Ozone Gas,Ozone Ref,Ozone Temp,Oxygen");
  logString("Millis,Stamp,Datetime,Temperature,Pressure,Humidity,Gas,Approx. Altitude,TVOC,eCO2,CFC,Benzene,Ozone Gas,Ozone Ref,Ozone Temp,Oxygen");    
}

void loop() {
  // delay for the amount of time we want between readings
  delay((LOG_INTERVAL -1) - (millis() % LOG_INTERVAL));

  String str = getAtmosphericData();
  logString(str);
  
  Wire.requestFrom(NANO, 4);
  Serial.print("> Geiger: ");
  Serial.println(Wire.read());
  
  Serial.println(getAtmosphericData());
  //runAtmosphere();
  //runSpaceFire();

  // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses a bunch of power and takes time
  if ((millis() - syncTime) < SYNC_INTERVAL) return;
  syncTime = millis();
}





