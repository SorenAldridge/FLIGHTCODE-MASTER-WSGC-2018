/*  DATA LOGGER CODE FOR THE ARDUINO MEGA ON BOARD THE 2018 ELIJAH BALLOON PAYLOAD
 *  
 *  Author: Quinlan Bock
 *  Date: July 8th 2018
 */

//Library Imports 
#include <Wire.h>
#include "SdFat.h"
#include "RTClib.h"
#include "Adafruit_BME680.h"
#include "Adafruit_SGP30.h"

const uint8_t chipSelect = 10;            // SD chip select pin.
const uint32_t SAMPLE_INTERVAL_MS = 1000; // Sample rate for the data in milliseconds

#define FILE_BASE_NAME "Data"             // Log file base name.  Must be six characters or less.
#define SEALEVELPRESSURE_HPA (1013.25)    // Change this number based upon the forcast that day

/***************************************************************************************************/

SdFat sd;             // File system object.
SdFile file;          // Log file.
RTC_PCF8523 rtc;      // Realtime clock object 
Adafruit_BME680 bme;  // I2C object for the bme680 sensor 
Adafruit_SGP30 sgp;   // I2C object for the sgp30 sensor

// Define Analog Pins for sensors
int CFC_Pin = 0;
int Benzene_Pin = 1;
int Ozone_Gas_Pin = 2;
int Ozone_Ref_Pin = 3;
int Ozone_Temp_Pin = 4;
int O2_Pin = 5;

// Time in micros for next data record.
uint32_t logTime;

/***************************************************************************************************/

// Function to write the header to the csv file
void writeHeader() {
  file.println("Millis,Stamp,Datetime,Temperature,Pressure,Humidity,Gas,Approx. Altitude,TVOC,eCO2,CFC,Benzene,Ozone Gas,Ozone Ref,Ozone Temp, O2");
}
// Error messages stored in flash.
#define error(msg) sd.errorHalt(F(msg))
// Function to record the data to a line of the csv file
void logData() {
  DateTime now = rtc.now();   // Get Datetime object from the RTC to use for

  // Check to see if BME sensor is working
  if (! bme.performReading()) {
    Serial.println("BME Measurement failed");
    return;
  }

  // Check to see if SGP sensor is working
  if (! sgp.IAQmeasure()) {
    Serial.println("SGP30 Measurement failed");
    return;
  }
  
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
  float O2 = analogRead(O2_Pin);

  // Write data to file.  Start with log time in micros.
  file.print(logTime/1000);
  file.print(",");

  // log time
  file.print(now.unixtime());    file.print(", "); file.print('"');
  file.print(now.year(), DEC);   file.print("/");
  file.print(now.month(), DEC);  file.print("/");
  file.print(now.day(), DEC);    file.print(" ");
  file.print(now.hour(), DEC);   file.print(":");
  file.print(now.minute(), DEC); file.print(":");
  file.print(now.second(), DEC); file.print('"');

  // print data
  file.print(", "); file.print(temp);
  file.print(", "); file.print(pressure);
  file.print(", "); file.print(humidity);
  file.print(", "); file.print(gas_resistance);
  file.print(", "); file.print(altitude);
  file.print(", "); file.print(TVOC);
  file.print(", "); file.print(eCO2);
  file.print(", "); file.print(CFC_reading);
  file.print(", "); file.print(Benzene_reading);
  file.print(", "); file.print(ozoneGas);
  file.print(", "); file.print(ozoneRef);
  file.print(", "); file.print(ozoneTemp);
  file.print(", "); file.print(O2);
  file.println();
}
//------------------------------------------------------------------------------
void logSetup() {
  const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
  char fileName[13] = FILE_BASE_NAME "00.csv";

  Serial.begin(9600);

  if (!bme.begin()) {
    Serial.println("BME680 sensor not found");
    while (1);
  }
  if (! sgp.begin()){
    Serial.println("SGP30 sensor not found");
    while (1);
  }

  // connect to RTC
  Wire.begin();  
  if (!rtc.begin()) {
    Serial.println("RTC failed");
  }
  if (! rtc.initialized()) {
    Serial.println("RTC is NOT running!");
  }

  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  
  // Initialize at the highest speed supported by the board that is
  // not over 50 MHz. Try a lower speed if SPI errors occur.
  if (!sd.begin(chipSelect, SD_SCK_MHZ(50))) {
    sd.initErrorHalt();
  }

  // Find an unused file name.
  if (BASE_NAME_SIZE > 6) {
    error("FILE_BASE_NAME too long");
  }
  while (sd.exists(fileName)) {
    if (fileName[BASE_NAME_SIZE + 1] != '9') {
      fileName[BASE_NAME_SIZE + 1]++;
    } else if (fileName[BASE_NAME_SIZE] != '9') {
      fileName[BASE_NAME_SIZE + 1] = '0';
      fileName[BASE_NAME_SIZE]++;
    } else {
      error("Can't create file name");
    }
  }
  if (!file.open(fileName, O_CREAT | O_WRITE | O_EXCL)) {
    error("file.open");
  }
  // Read any Serial data.
  do {
    delay(10);
  } while (Serial.available() && Serial.read() >= 0);

  // Write data header.
  writeHeader();

  // Start on a multiple of the sample interval.
  logTime = micros()/(1000UL*SAMPLE_INTERVAL_MS) + 1;
  logTime *= 1000UL*SAMPLE_INTERVAL_MS;
}
//------------------------------------------------------------------------------
void logLoop() {
  // Time for next record.
  logTime += 1000UL*SAMPLE_INTERVAL_MS;

  // Wait for log time.
  int32_t diff;
  do {
    diff = micros() - logTime;
  } while (diff < 0);

  // Check for data rate too high.
  if (diff > 10) {
    error("Missed data record");
  }

  logData();

  // Force data to SD and update the directory entry to avoid data loss.
  if (!file.sync() || file.getWriteError()) {
    error("write error");
  }

  if (Serial.available()) {
    // Close file and stop.
    file.close();
    Serial.println(F("Done"));
    SysCall::halt();
  }
}
