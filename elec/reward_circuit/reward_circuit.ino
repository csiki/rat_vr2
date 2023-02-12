#include <SafeString.h>
#include <BufferedOutput.h>
#include <SafeStringReader.h>
#include <Unistep2.h>

createSafeStringReader(sfReader, 80, '\n');  // create a SafeString reader with max Cmd Len 15 and delimiters space, comma, Carrage return and Newline

//Example of using BufferedOutput to release bytes when there is space in the Serial Tx buffer, extra buffer size 80
createBufferedOutput(bufferedOut, 80, DROP_UNTIL_EMPTY);
Unistep2 stepper(2, 4, 7, 8, 4096, 1000);

unsigned long currentMillis = 0;
unsigned long pMillisfeed = 0;
unsigned long pMillisValve = 0;
unsigned long pMillispumpOR = 0;
unsigned long pMillisblowL = 0;
unsigned long pMillisblowR = 0;
const long feedinterval = 1000;
int valveinterval = 0;
int pumpORinterval = 0;
int blowlinterval = 0;
int blowrinterval = 0;

int rawValue;          // A/D readings
int offset = 410;      // zero pressure adjust
int fullScale = 9630;  // max pressure (span) adjust
float pressure;        // final pressure
float pressureSP = 92; // pressure Setpoint

int valvePin = 13;
int motor1pin1 = 5;
int motor1pin2 = 3;
int motor2pin3 = 9;
int motor2pin4 = 10;
int motor3pin5 = 12;
int motor3pin6 = 11;
int turns = 0;         // Stepper Turns
int j = 0;             // Stepper control flag

bool pumpORflag = false;

// the setup function runs once when you press reset or power the board
void setup() {
    Serial.begin(57600);

    pinMode(valvePin, OUTPUT);
    pinMode(motor1pin1, OUTPUT);
    pinMode(motor1pin2, OUTPUT);
    pinMode(motor2pin3, OUTPUT);
    pinMode(motor2pin4, OUTPUT);
    pinMode(motor3pin5, OUTPUT);
    pinMode(motor3pin6, OUTPUT);

//    for (int i = 10; i > 0; i--) {
//        Serial.print(i);
//        Serial.print(' ');
//        delay(500);
//    }

//    Serial.println();
//    // SafeString::setOutput(Serial); //uncomment this to enable error msgs
//    Serial.println("Data format: $DOOM,[Valve Open Millisec(int)],[Pressure SetPoint(float)],[Pump Override Control Millisec(int)],[Left Blow Millisec(int)],[Right Blow Millisec(int)],[Stepper Turns(int)]*CheckSum8Xor");
//    Serial.println(" e.g.: $DOOM,5000,94.4,5000,2500,3500,3*2C");

    bufferedOut.connect(Serial);  // connect bufferedOut to Serial
    sfReader.connect(bufferedOut);
    sfReader.echoOn();         // echo goes out via bufferedOut
    sfReader.setTimeout(100);  // set 100ms == 0.1sec non-blocking timeout
}

bool checkSum(SafeString &msg) {
    int idxStar = msg.indexOf('*');
    cSF(sfCheckSumHex, 2);
    msg.substring(sfCheckSumHex, idxStar + 1);  // next 2 chars SafeString will complain and return empty substring if more than 2 chars
    long sum = 0;
    if (!sfCheckSumHex.hexToLong(sum)) {
        return false;  // not a valid hex number
    }
    for (size_t i = 1; i < idxStar; i++) {  // skip the $ and the *checksum
        sum ^= msg[i];
    }
    return (sum == 0);
}
//parsing approprate values
void parseFeedValve(SafeString &dataField) {
    int msec = 0;
    if (!dataField.toInt(msec)) {
        return;  // invalid
    }
    valveinterval += msec;  // TODO changed to +=, TEST IF WORKS
}
void parseFeedPressure(SafeString &dataField) {
    float msec = 0;
    if (!dataField.toFloat(msec)) {
        return;  // invalid
    }
    pressureSP += msec;
}
void parseFeedPump(SafeString &dataField) {
    int msec = 0;
    if (!dataField.toInt(msec)) {
        return;  // invalid
    }
    pumpORinterval += msec;
    pumpORflag = true;
}
void parseBlowLeft(SafeString &dataField) {
    int msec = 0;
    if (!dataField.toInt(msec)) {
        return;  // invalid
    }
    blowlinterval += msec;
}
void parseBlowRight(SafeString &dataField) {
    int msec = 0;
    if (!dataField.toInt(msec)) {
        return;  // invalid
    }
    blowrinterval += msec;
}
void parseMixerTurns(SafeString &dataField) {
    int msec = 0;
    if (!dataField.toInt(msec)) {
        return;  // invalid
    }
    turns += msec;
}

// just leaves existing values unchanged if new ones are not valid
// returns false if msg Not Active
bool parseDOOM(SafeString &msg) {
    cSF(sfField, 11);               // temp SafeString to received fields, max field len is <11;
    char delims[] = ",*";           // fields delimited by , or *
    bool returnEmptyFields = true;  // return empty field for ,,
    int idx = 0;
    idx = msg.stoken(sfField, idx, delims, returnEmptyFields);
    if (sfField == "$STOP") {  // stop everything
        valveinterval = pressureSP = pumpORinterval = blowlinterval = blowrinterval = turns = 0;
        pumpORflag = false;
        return true;
    } else if (sfField != "$DOOM") {  // first field should be $DOOM else called with wrong msg
        return false;
    }
    idx = msg.stoken(sfField, idx, delims, returnEmptyFields);
    parseFeedValve(sfField);
    idx = msg.stoken(sfField, idx, delims, returnEmptyFields);
    parseFeedPressure(sfField);
    idx = msg.stoken(sfField, idx, delims, returnEmptyFields);
    parseFeedPump(sfField);
    idx = msg.stoken(sfField, idx, delims, returnEmptyFields);
    parseBlowLeft(sfField);
    idx = msg.stoken(sfField, idx, delims, returnEmptyFields);
    parseBlowRight(sfField);
    idx = msg.stoken(sfField, idx, delims, returnEmptyFields);
    parseMixerTurns(sfField);
    return true;
}

void printResponse() {
    Serial.print(F(" > > > "));
    Serial.print(F("  "));
    Serial.print("VLV:");
    Serial.print(valveinterval);
    Serial.print("|PSP:");
    Serial.print(pressureSP);
    Serial.print("|PMP:");
    Serial.print(pumpORinterval);
    Serial.print("|BL:");
    Serial.print(blowlinterval);
    Serial.print("|BR:");
    Serial.print(blowrinterval);
    Serial.print("|TS:");
    Serial.print(turns);
    Serial.print("|P:");
    Serial.print(pressure);
    Serial.print(" kPa");
    Serial.println();
}

void feedValve(){
    pMillisValve = currentMillis;
    digitalWrite(valvePin, HIGH);
}
void pumpOverride(){
    pMillispumpOR = currentMillis;
    digitalWrite(motor1pin1, HIGH);
    digitalWrite(motor1pin2, LOW);
}
void blowLeft(){
    pMillisblowL = currentMillis;
    digitalWrite(motor2pin3, HIGH);
    digitalWrite(motor2pin4, LOW);
}
void blowRight() {
    pMillisblowR = currentMillis;
    digitalWrite(motor3pin5, HIGH);
    digitalWrite(motor3pin6, LOW);
}
void mixerControl(){
    j = 0;
}

void timerHandler(){
    if (currentMillis - pMillisValve >= valveinterval) {
        digitalWrite(valvePin, LOW);
    }
    if (pumpORflag && (currentMillis - pMillispumpOR) >= pumpORinterval) {
        digitalWrite(motor1pin1, LOW);
        digitalWrite(motor1pin2, LOW);
        pumpORflag = false;
    }
    if (currentMillis - pMillisblowL >= blowlinterval) {
        digitalWrite(motor2pin3, LOW);
        digitalWrite(motor2pin4, LOW);
    }
    if (currentMillis - pMillisblowR >= blowrinterval) {
        digitalWrite(motor3pin5, LOW);
        digitalWrite(motor3pin6, LOW);
    }
    if (stepper.stepsToGo() == 0 && j < turns) {
        for (int i = 0; i <= 4096; i++) {
            stepper.move(i);
        }
        j++;
    }
}

void processUserInput() {
    if (sfReader.read()) {
        sfReader.trim();            // remove and leading/trailing white space
        if (!checkSum(sfReader)) {  // is the check sum OK
            Serial.print("bad checksum : ");
            Serial.println(sfReader);
        } else {                                  // check sum OK so select msgs to process
            if (sfReader.startsWith("$DOOM,") || sfReader.startsWith("$STOP,")) {  // this is the one we want
                if (parseDOOM(sfReader)) {
                    feedValve();
                    pumpOverride();
                    blowLeft();
                    blowRight();
                    printResponse();
                    mixerControl();
                }
            } else {  /* some other msg */ }
        }
    }
}

void feedSystem() {
    if (currentMillis - pMillisfeed >= feedinterval) {
        pMillisfeed = currentMillis;
        rawValue = 0;
        for (int x = 0; x < 10; x++) rawValue = rawValue + analogRead(A0);
        pressure = (rawValue - offset) * 700.0 / (fullScale - offset);  // pressure conversion

        // Serial.print("Raw A/D is  ");
        // Serial.print(rawValue);
        // Serial.print("   Pressure is  ");
        // Serial.print(pressure, 1);  // one decimal places
        // Serial.println("  kPa");
    }
    if (!pumpORflag && (pressure < pressureSP)) {
        digitalWrite(motor1pin1, HIGH);
        digitalWrite(motor1pin2, LOW);
    } else if (!pumpORflag) {
        digitalWrite(motor1pin1, LOW);
        digitalWrite(motor1pin2, LOW);
    }
}

// the loop function runs over and over again forever
void loop() {
    currentMillis = millis();
    stepper.run();  // non-blocking stepper driver function
    timerHandler(); // all controls timing
    bufferedOut.nextByteOut();  // call this one or more times each loop() to release buffered chars
    processUserInput();  // parsing control parameters
    feedSystem();  // background automotion pressurising system
}
