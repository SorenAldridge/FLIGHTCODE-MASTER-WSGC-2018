#include <Wire.h>

#define LOG_PERIOD 30000  //Logging period in ms
#define MAX_PERIOD 60000  //Maximum logging period without modifying

volatile unsigned long counts;     //variable for GM Tube events
unsigned long cpm;        //variable for CPM

unsigned int multiplier = MAX_PERIOD / LOG_PERIOD;  //variable for calculation CPM 

unsigned long previousMillis;  //variable for time measurement

void setup() {
  pinMode(2, INPUT); 
  //attachInterrupt(digitalPinToInterrupt(2), tube, FALLING);
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onRequest(requestEvent); // register event
  counts = 0;
  cpm = 0;
  Serial.begin(9600);
  
}

void loop() {
  unsigned long currentMillis = millis();
  if(digitalRead(2)==HIGH)
  {
    counts++;
    delay(30);
  }
  if(currentMillis - previousMillis > 30000)  // LOG_PERIOD = 30 seconds
  {
    previousMillis = currentMillis;
    cpm = counts * 2;// counts for 30- seconds * 2 = cpm
    Serial.println(counts);
    Serial.println(cpm);     
    requestEvent;  
    counts = 0;    
  }   
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() {
  Wire.write(cpm); // respond with message of 6 bytes

}


void tube(){       
  counts+1;
}
