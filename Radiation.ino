
int pushButton = 2;
int count;


void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  // make the pushbutton's pin an input:
  pinMode(pushButton, INPUT);
  count=0;
}


void loop() {
  int buttonState = digitalRead(pushButton);
  if (buttonState == HIGH) {     
      count++;
      delay(150);
  } 
  Serial.println(count);
  delay(1);        
}
