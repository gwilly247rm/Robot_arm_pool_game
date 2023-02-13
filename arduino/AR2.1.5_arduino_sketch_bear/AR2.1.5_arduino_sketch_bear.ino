#include <Wire.h>  //Include Wire library for using I2C functions

#define MCP4725 0x60  // MCP4725 address as 0x61 Change yours accordingly

unsigned int adc;
byte buffer[3];

String inData;
String function;

const int Input2 = 2;

const int Output3 = 3;
const int Output4 = 4;

const int Output38 = 38;
const int Output39 = 39;
const int Output40 = 40;
const int Output41 = 41;
const int Output42 = 42;
const int Output43 = 43;
const int Output44 = 44;
const int Output45 = 45;
const int Output46 = 46;
const int Output47 = 47;
const int Output48 = 48;
const int Output49 = 49;
const int Output50 = 50;
const int Output51 = 51;
const int Output52 = 52;
const int Output53 = 53;

void setup() {
    // run once:
    Serial.begin(9600);
    Wire.begin();

    pinMode(Input2, INPUT);
    pinMode(Output3, OUTPUT);
    pinMode(Output4, OUTPUT);

    pinMode(Output38, OUTPUT);
    pinMode(Output39, OUTPUT);
    pinMode(Output40, OUTPUT);
    pinMode(Output41, OUTPUT);
    pinMode(Output42, OUTPUT);
    pinMode(Output43, OUTPUT);
    pinMode(Output44, OUTPUT);
    pinMode(Output45, OUTPUT);
    pinMode(Output46, OUTPUT);
    pinMode(Output47, OUTPUT);
    pinMode(Output48, OUTPUT);
    pinMode(Output49, OUTPUT);
    pinMode(Output50, OUTPUT);
    pinMode(Output51, OUTPUT);
    pinMode(Output52, OUTPUT);
    pinMode(Output53, OUTPUT);

    pinMode(A0, INPUT);
    pinMode(A1, INPUT);
    pinMode(A2, INPUT);

    pinMode(Input2, INPUT);

    analogWrite(Output3, 0);
    analogWrite(Output4, 0);

    digitalWrite(Output38, HIGH);
    digitalWrite(Output39, HIGH);
    digitalWrite(Output40, HIGH);
    digitalWrite(Output41, HIGH);
    digitalWrite(Output42, HIGH);
    digitalWrite(Output43, HIGH);
    digitalWrite(Output44, HIGH);
    digitalWrite(Output45, HIGH);

    digitalWrite(Output46, HIGH);
    digitalWrite(Output47, HIGH);
    digitalWrite(Output48, HIGH);
    digitalWrite(Output49, HIGH);
    digitalWrite(Output50, HIGH);
    digitalWrite(Output51, HIGH);
    digitalWrite(Output52, HIGH);
    digitalWrite(Output53, HIGH);
    dac(0);
}

void loop() {
    while (Serial.available() > 0) {
  char recieved = Serial.read();
  inData += recieved;
  // Process message when new line character is recieved
  if (recieved == '\n') {
      String function = inData.substring(0, 2);

      //-----COMMAND TO WAIT
      // TIME---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "WT") {
    int WTstart = inData.indexOf('S');
    float WaitTime = inData.substring(WTstart + 1).toFloat();
    int WaitTimeMS = WaitTime * 1000;
    delay(WaitTimeMS);
      }

      //-----COMMAND SET OUTPUT
      // ON---------------------------------------------------
      //-----------------------------------------------------------------------
      else if (function == "ON") {
    int ONstart = inData.indexOf('X');
    int outputNum = inData.substring(ONstart + 1).toInt();
    digitalWrite(outputNum, LOW);
      }
      //-----COMMAND SET OUTPUT
      // OFF---------------------------------------------------
      //-----------------------------------------------------------------------
      else if (function == "OF") {
    int ONstart = inData.indexOf('X');
    int outputNum = inData.substring(ONstart + 1).toInt();
    digitalWrite(outputNum, HIGH);
      }
      else if (function == "CV") {
    int ONstart = inData.indexOf('X');
    int outputNum = inData.substring(ONstart + 1).toInt();
    dac(outputNum);
      }
      //-----COMMAND SET VOLTAGE
      // CONTROL---------------------------------------------------
      //-----------------------------------------------------------------------
      else if (function == "VC") {
    int VCpin = inData.indexOf('P');
    int VCvalue = inData.indexOf('L');
    int outputPin = inData.substring(VCpin + 1, VCvalue).toInt();
    int outputNum = inData.substring(VCvalue + 1).toInt();
    analogWrite(outputPin, outputNum);
      }
      //-----COMMAND TO WAIT INPUT
      // ON---------------------------------------------------
      //-----------------------------------------------------------------------
      else if (function == "WI") {
    int WIstart = inData.indexOf('N');
    int InputNum = inData.substring(WIstart + 1).toInt();
    while (digitalRead(InputNum) == LOW) {
        delay(100);
    }
      }
      //-----COMMAND TO WAIT INPUT
      // OFF---------------------------------------------------
      //-----------------------------------------------------------------------
      else if (function == "WO") {
    int WIstart = inData.indexOf('N');
    int InputNum = inData.substring(WIstart + 1).toInt();

    // String InputStr =  String("Input" + InputNum);
    // uint8_t Input = atoi(InputStr.c_str ());
    while (digitalRead(InputNum) == HIGH) {
        delay(100);
    }
      }

      inData = "";  // Clear recieved buffer
  }
    }
}

void dac(float Air) {
    buffer[0] = 0b01000000;
    // adc = analogRead(A0) * 4;

    float InVolt;

    InVolt = Air / 1.87;
    adc = (int)(InVolt / (5.0 / 4096.0));  // 0~4096;

    buffer[1] = adc >> 4;
    buffer[2] = adc << 4;

    Wire.beginTransmission(
  MCP4725);  // Joins I2C bus with MCP4725 with 0x61 address

    Wire.write(buffer[0]);  // Sends the control byte to I2C
    Wire.write(buffer[1]);  // Sends the MSB to I2C
    Wire.write(buffer[2]);  // Sends the LSB to I2C

    Wire.endTransmission();  // Ends the transmission
}
