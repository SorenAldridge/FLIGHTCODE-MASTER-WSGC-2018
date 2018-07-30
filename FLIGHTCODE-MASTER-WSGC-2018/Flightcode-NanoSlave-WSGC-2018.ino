#include <Wire.h>

#define LOG_PERIOD 30000  //Logging period in ms
#define MAX_PERIOD 60000  //Maximum logging period without modifying

volatile unsigned long counts;     //variable for GM Tube events
unsigned long cpm;        //variable for CPM

unsigned int multiplier = MAX_PERIOD / LOG_PERIOD;  //variable for calculation CPM 

unsigned long previousMillis;  //variable for time measurement

void setup() {
  Wire.begin(); // join i2c bus (address optional for master)
    counts = 0;
  cpm = 151;
  multiplier = MAX_PERIOD / LOG_PERIOD;      //calculating multiplier
  Serial.begin(9600);
  attachInterrupt(0, tube_impulse, FALLING); //define external interrupts
}

byte x = 0;

void loop() {

    unsigned long currentMillis = millis();
  if(currentMillis - previousMillis > LOG_PERIOD)  // LOG_PERIOD = 30 seconds
  {
    previousMillis = currentMillis;
    cpm = counts * multiplier;// counts for 30- seconds * 2 = cpm
    Wire.beginTransmission(8); // transmit to device        
    Wire.write(cpm);              
    Wire.endTransmission();    // stop transmitting
    counts = 0;        
  }
  
}


void tube_impulse(){       
  counts++;
}



