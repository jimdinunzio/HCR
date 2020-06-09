/*
 /*
 # This Sample code is for testing the DC Motor Driver 2x15A_lite module.

 # Editor : Phoebe
 # Date   : 2012.11.6
 # Ver    : 0.1
 # Product: DC Motor Driver 2x15A_lite
 # SKU    : DRI0018

 # Description:
 # Drive 2 motors with this DC Motor Driver module

 # Hardwares:
 1. Arduino UNO
 2. DC Motor Driver 2x15A_lite
 3. DC motors x2

 #Steps:
 1.Connect the M1_PWM & M2_PWM to UNO digital 5 & 6
 2.Connect the M1_EN & M2_EN to UNO digital 4 & 7
 3.Connect +5V & GND to UNO 5V & GND

 # Function for current sense and diagnosis,if you want to use
 please connect the IS pins to Arduino
 Connect LA_IS and RA_IS to UNO digital 2 at the same time
 Connect LB_IS and RB_IS to UNO digital 3 at the same time
 */

const int EL = 5;     //Left (M1) Speed Control
const int ER = 6;     //Right(M2) Speed Control
const int ML = 4;     //Left (M1) Direction Control
const int MR = 7;     //Right(M2) Direction Control
int counter=0;

const int x1Pos = 10; // horizontal position for buttton 1 
const int x2Pos = 210; // horizontal position for buttton 2 
const int xPos = 110; // horizontal position for buttons 3,4
const int yPos = 90; // vertical position of buttons 1,2,5
const int y1Pos = 10; // vertical position of button 3
const int y4Pos = 170; // vertical position of button 4
const int width = 70;
const int height = 70;
const int radius = 15;
const int alignment = 0; // 0=centered, 1=right, 2=left, 3=bottom, 4=top 
const int option = 1; // 1=draw, 2=disabled, 3=toggle pressed, 4=toggle not pressed,
// 5=toggle pressed disabled, 6=toggle not pressed disabled.
int touch=0;

#include <ezLCDLib.h>


enum RotDir {
  RD_CW = LOW,
  RD_CCW = HIGH
};

ezLCD3 lcd;

void stop(void)                    //Stop
{
  lcd.Debug("STOP\r\n");
  digitalWrite(EL,0);
  digitalWrite(ML,RD_CW);
  digitalWrite(ER,0);
  digitalWrite(MR,RD_CW);
}
void advance(char a,char b)          //Move forward
{
  lcd.Debug("Advance\r\n");
  analogWrite (EL,a);      //PWM Speed Control
  digitalWrite(ML,RD_CW);
  analogWrite (ER,b);
  digitalWrite(MR,RD_CCW);
}
void back_off (char a,char b)          //Move backward
{
  lcd.Debug("Back off\r\n");
  analogWrite (EL,a);
  digitalWrite(ML,RD_CCW);
  analogWrite (ER,b);
  digitalWrite(MR,RD_CW);
}
void turn_L (char a,char b)             //Turn Left
{
  lcd.Debug("Turn left\r\n");
  analogWrite (EL,a);
  digitalWrite(ML,RD_CCW);
  analogWrite (ER,b);
  digitalWrite(MR,RD_CCW);
}
void turn_R (char a,char b)             //Turn Right
{
  lcd.Debug("Turn right\r\n");
  analogWrite (EL,a);
  digitalWrite(ML,RD_CW);
  analogWrite (ER,b);
  digitalWrite(MR,RD_CW);
}
void current_sense()                  // current sense and diagnosis
{
  int val1=digitalRead(2);
  int val2=digitalRead(3);
  if(val1==HIGH || val2==HIGH){
    counter++;
    if(counter==3){
      counter=0;
      lcd.Debug("Warning\r\n");
    }
  }
}

enum MoveDirection {
  MD_LEFT = 1,
  MD_RIGHT,
  MD_FORWARD,
  MD_REVERSE,
  MD_STOP
};

void setup(void)
{
  lcd.begin( );
  lcd.cls( BLACK, WHITE );
  lcd.font("0");

  int i;
  for(i=4;i<=7;i++)
    pinMode(i, OUTPUT);
  lcd.Debug("Run keyboard control\r\n");

  lcd.print("Motor\rController\rTest");
  lcd.fontw( 1, "sans24" );
  lcd.theme( 1, 9, 3, 4, 0, 0, 8, 8, 8, 1, 1 );
  lcd.theme( 2, 5, 20, 3, 3, 3, 4, 4, 4, 2, 1 );
  lcd.string( MD_LEFT, "Left" ); // stringId 1
  lcd.string( MD_RIGHT, "Right" ); // stringId 2
  lcd.string( MD_FORWARD, "Forward"); // stringId 3
  lcd.string( MD_REVERSE, "Reverse"); // stringId 4
  lcd.string( MD_STOP, "STOP");
  
  lcd.button( MD_LEFT, x1Pos, yPos, width, height, option, alignment, radius, 1, 1 );
  lcd.button( MD_RIGHT, x2Pos, yPos, width, height, option, alignment, radius, 1, 2 );
  lcd.button( MD_FORWARD, xPos, y1Pos, width, height, option, alignment, radius, 1, 3 );
  lcd.button( MD_REVERSE, xPos, y4Pos, width, height, option, alignment, radius, 1, 4 );
  lcd.button( MD_STOP, xPos, yPos, width, height, option, alignment, radius, 2, 5 );

  // Motor controller pins
  digitalWrite(EL,LOW);
  digitalWrite(ER,LOW);
  pinMode(2,INPUT);
  pinMode(3,INPUT);
}

unsigned long runTime = 0;

void loop(void)
{
//  static unsigned long timePoint = 0;    // current sense and diagnosis,if you want to use this
//   if(millis() - timePoint > 1000){       //function,please show it & don't forget to connect the IS pins to Arduino
//   current_sense();
//   timePoint = millis();
//   }
  
  touch = lcd.wstack(0);
  switch(touch)
  {
    case MD_LEFT:
      runTime = millis();
      turn_L (100,100);
      break;
    case MD_RIGHT:
      runTime = millis();
      turn_R (100,100);
      break;
    case MD_FORWARD:
      runTime = millis();
      advance (128,128);   //move forward
      break;
    case MD_REVERSE:
      runTime = millis();
      back_off (128,128);   //move back
      break;
    case MD_STOP:
      runTime = 0;
      stop();
      break;
  }
  
  if(runTime != 0 && millis() - runTime > 2000) {
    stop();
    runTime = 0;
  }
  
  delay(20);
}
