/*
**
 **  File    : ESC_Sketch.pde
 **  Author  : J.Hirvinen, jani@jdr.........es.com
 **  Version : v1.0, 30.01.2012
 **
 ***************************************************************
 ** Description
 **
 ** Simple program to sent PWM values for Brushless motor ESCs
 **
 */
 
///////////////////////////////////////////////////////////////////////////////
// include the library codes
 
 
///////////////////////
// Global variables
#define LED_OUT 13
 
const int outputPin = 3; // Connect ESC PWM input to D3
const int minESCoutput = 126; // minimum ESC output
const int maxESCoutput = 250; // maximum ESC output
int userInput = 0;
int outputPower = 0;
int ESCoutput = 1000;
int analogWriteOutput;
 
void setup() {
  Serial.begin(115200);
  analogWrite(outputPin, minESCoutput);
}
 
void loop() {
 
  Serial.println("ESC Tester Menu");
  Serial.println("---------------");
  Serial.println("0 =   0% Output");
  Serial.println("1 =  25% Output");
  Serial.println("2 =  50% Output");
  Serial.println("3 =  75% Output");
  Serial.println("4 = 100% Output");
  Serial.println("---------------");
  Serial.print("Enter Motor Output Value: ");
  while (Serial.available() == 0)
    delay(100);
  userInput = Serial.read();
 
  switch (userInput) {
    case 49:
      outputPower = 25;
      Serial.println('1');
      break;
    case 50:
      outputPower = 50;
      Serial.println('2');
      break;
    case 51:
      outputPower = 75;
      Serial.println('3');
      break;
    case 52:
      outputPower = 100;
      Serial.println('4');
      break;
    default:
      outputPower = 0;
      Serial.println('0');
      break;
  }
 
  ESCoutput = map(outputPower, 0, 100, minESCoutput, maxESCoutput);
 
  analogWriteOutput = map(outputPower, 0, 100, 1000, 2000);
  analogWrite(outputPin, ESCoutput);
 
  Serial.print("ESC output set to ");
  Serial.print(outputPower);
  Serial.println("%");
 
  Serial.print("PWM output value of ");
  Serial.print(analogWriteOutput);
  Serial.println(" microseconds");
  Serial.println();
  Serial.println();
 
}
