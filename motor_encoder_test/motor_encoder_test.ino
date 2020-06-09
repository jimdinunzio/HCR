/*
Pay attention to the interrupt pin,please check which microcontroller you use.
http://arduino.cc/en/Reference/AttachInterrupt
*/

enum WheelDirection {
  WD_CW,
  WD_CCW
};

#include <ezLCDLib.h>

ezLCD3 lcd;

// Encoder 0 is on LEFT motor
// Encoder 1 is on RIGHT motor

//The sample code for driving one way motor encoder
const byte encoderLpinA = 2;//A pin -> the interrupt pin 2
const byte encoderLpinB = 12;//B pin -> the digital pin 12
const byte encoderRpinA = 3;//A pin -> the interrupt pin 3
const byte encoderRpinB = 13;//B pin -> the digital pin 13

volatile byte encoderLPinALast = LOW;
volatile byte encoderRPinALast = LOW;
volatile int durationL = 0;//the number of the pulses for left encoder
volatile int durationR = 0;//the number of the pulses for right encoder
volatile WheelDirection dirL;//the rotation direction of left wheel
volatile WheelDirection dirR;//the rotation direction of right wheel

void setup()
{
  lcd.begin( );
  lcd.cls( BLACK, WHITE );
  lcd.font("0");
  encoderInit();//Initialize the module
}

void loop()
{
  lcd.print("Pulse left:");
  if(durationL < 0)
    lcd.print("-");
  lcd.print(abs(durationL), DEC);
  lcd.print(", right:");
  if(durationR < 0)
    lcd.print("-");
  lcd.println(abs(durationR), DEC);

  durationL = 0;
  durationR = 0;
  delay(100);
}

void encoderInit()
{
  dirL = WD_CCW;//default -> Forward
  dirR = WD_CW;//default -> Forward
  pinMode(encoderLpinB,INPUT);
  pinMode(encoderRpinB,INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderLpinA), wheelSpeedL, CHANGE);//int.0
  attachInterrupt(digitalPinToInterrupt(encoderRpinA), wheelSpeedR, CHANGE);//int.1
}

void wheelSpeedL()
{
  int Lstate = digitalRead(encoderLpinA);
  if((encoderLPinALast == LOW) && Lstate == HIGH)
  {
    int val = digitalRead(encoderLpinB);
    if(val == LOW && dirL == WD_CCW)
    {
      dirL = WD_CW; // Forward
    }
    else if(val == HIGH && dirL == WD_CW)
    {
      dirL = WD_CCW; // Reverse
    }
  }
  encoderLPinALast = Lstate;

  if(dirL == WD_CW) durationL++;
  else durationL--;
}

void wheelSpeedR()
{
  int Rstate = digitalRead(encoderRpinA);
  if((encoderRPinALast == LOW) && Rstate == HIGH)
  {
    int val = digitalRead(encoderRpinB);
    if(val == LOW && dirR == WD_CCW)
    {
      dirR = WD_CW; // Reverse
    }
    else if(val == HIGH && dirR == WD_CW)
    {
      dirR = WD_CCW; // Forward
    }
  }
  encoderRPinALast = Rstate;

  if(dirR == WD_CCW) durationR++;
  else durationR--;
}
