#include <Servo.h>


Servo servo;
int servoPin = 6;
int servoOffPos = 120;
int servoOnPos = 180;
int lastSwitched = 0;
int switchh = servoOffPos;

int rotCLK = A1;
int rotDT = A2;
int rotSW = A3;
int rotDebounceDelay = 5;
byte rotPos = 0, rotLastPos = 0, rotLastState;

//int rotCounter = 0;
//int rotCurrentStateCLK;
//int rotLastStateCLK;
//String rotCurrentDir = "";

void setup() {
    Serial.begin(57600);
    
    pinMode(servoPin, OUTPUT);
    servo.attach(servoPin);
    servo.write(servoOffPos);

    pinMode(rotDT, INPUT);
    pinMode(rotSW, INPUT_PULLUP);
    pinMode(rotCLK, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(rotCLK), encoder, CHANGE);
}

void loop() {

    if (millis() - lastSwitched > 2000)
    {
        lastSwitched = millis();
        if (switchh == servoOffPos)
            switchh = servoOnPos;
        else
            switchh = servoOffPos;

        servo.write(switchh);
    }

    if(rotPos != rotLastPos)
    {
      Serial.println(rotPos);
      rotLastPos = rotPos;
    }
   
    // Check if encoder is pressed
    if(digitalRead(rotSW) == false)
      Serial.println("Pressed!");

}

// Interrupt service routine to read the encoder state
void encoder()
{
  // Wait for encoder contacts to settle
  delay(rotDebounceDelay);
 
  // Read the encoder outputs
  byte rotState = (digitalRead(rotDT) << 1) | digitalRead(rotCLK);
 
  // If the state has changed then update the counter
  if(rotState != rotLastState)
    (rotState == 3 || rotState == 0) ? rotPos-- : rotPos++;
 
  rotLastState = rotState;
}
