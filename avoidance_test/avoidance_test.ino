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

const int MIN_POWER = 33;
const int R_POWER = 50;
const int L_POWER = 50;

unsigned long runTime = 0;

enum WheelDirection {
  WD_CW,
  WD_CCW
};

enum MoveDirection {
  MD_STOPPED,
  MD_FORWARD,
  MD_REVERSE,
  MD_LEFT,
  MD_RIGHT
};

MoveDirection moveDirection;

// HCR 
// The motor could output 1469 pulse feedback signals per rotation. Wheel diameter is 13.4CM. 
// We can calculate that one meter is equivalent to 2.38 rotations. 
// Therefore, pulses per meter is 1469 * 2.38 = 3946 (set in motor.h)
const int PPM = 3946;
const int WB = 276; // mm

// half rotation = 434mm = 1711 pulses
const int HALF_TURN = 1711;

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

const int EL = 5;     //Left (M1) Speed Control
const int ER = 6;     //Right(M2) Speed Control
const int ML = 4;     //Left (M1) Direction Control
const int MR = 7;     //Right(M2) Direction Control
int counter=0;

// Bumper pins
const int BMP_L = 8; 
const int BMP_C = 9;
const int BMP_R = 10;
bool bmp_l_state = HIGH;
bool bmp_c_state = HIGH;
bool bmp_r_state = HIGH;

const byte MotorLeft = 0;
const byte MotorRight = 1;
const byte MotorMaster = MotorLeft;

enum MotorState {
  MS_Idle,
  MS_Running
};

enum MotorSyncMode {
  MSM_RToL,  // Left is the master, Right is the slave
  MSM_LToR, // Right is the master, Left is the slave
  MSM_None // off
};

enum WidgetId {
  WID_LEFT = 1,
  WID_RIGHT,
  WID_FORWARD,
  WID_REVERSE,
  WID_STOP,
  WID_LTICKS,
  WID_RTICKS,
  WID_LPWR,
  WID_RPWR
};

int motorSyncRatio = 100;
MotorSyncMode motorSyncMode = MSM_RToL;
int motorEncoderTarget[2] = {0, 0};
byte motorPower[2] = {0, 0};
MotorState motorState = MS_Idle;
unsigned long lastMotorUpdate = 0;
int prev_eI = 0;
int prev_eP = 0;
const float kP = 0.2;
const float kI = 0.0;
const float kD = 0.0;

void updateMotors()
{
  int eP, eI, eD;
  int powerAdjustment;
  byte correctedMotorPower[2];
  float targetAtten = 1.0;
  
  unsigned long dt = millis() - lastMotorUpdate;
  if (motorSyncMode == MSM_RToL) 
  {
    if(motorPower[MotorLeft] == 0) {
      stop();
      return;
    } 
    eP = durationL - durationR;
    if(motorEncoderTarget[MotorLeft] > 0) {
      int remaining = motorEncoderTarget[MotorLeft] - durationL;
      if(remaining > 0) {
        if(remaining < HALF_TURN) {
          targetAtten = sqrt((float)remaining / (float)HALF_TURN);
        }
      } else {
        motorPower[MotorLeft] = 0;
        motorEncoderTarget[MotorLeft] = 0;
        stop();
        return;
      }
    }
  }
  else if (motorSyncMode == MSM_LToR)
  {
    if(motorPower[MotorRight] == 0) {
      stop();
      return;
    }
    eP = durationR - durationL;
    if(motorEncoderTarget[MotorRight] > 0) {
      int remaining = motorEncoderTarget[MotorRight] - durationR;
      if(remaining > 0) {
        if(remaining < HALF_TURN) {
          targetAtten = sqrt((float)remaining / (float)HALF_TURN);
        }
      } else {
        motorPower[MotorRight] = 0;
        motorEncoderTarget[MotorRight] = 0;
        stop();
        return;
      }
    }
  }
  
  eI = prev_eI + eP * dt;
  eD = ( eP - prev_eP ) / dt;
  
  powerAdjustment = kP * eP + kI * eI + kD * eD;
  
  if (motorSyncMode == MSM_RToL) 
  {
    correctedMotorPower[MotorRight] = max(MIN_POWER, targetAtten * (float)constrain(motorPower[MotorLeft] + powerAdjustment / 2, 0, 255));
    correctedMotorPower[MotorLeft] = max(MIN_POWER, targetAtten * (float)constrain(motorPower[MotorLeft] - powerAdjustment / 2, 0, 255));
  }
  else if (motorSyncMode = MSM_LToR)
  {
    correctedMotorPower[MotorLeft] = max(MIN_POWER, targetAtten * (float)constrain(motorPower[MotorRight] + powerAdjustment / 2, 0, 255));
    correctedMotorPower[MotorRight] = max(MIN_POWER, targetAtten * (float)constrain(motorPower[MotorLeft] - powerAdjustment / 2, 0, 255));
  }
    
  advance(correctedMotorPower[MotorLeft], correctedMotorPower[MotorRight]);
  
  prev_eP = eP;
  prev_eI = eI;
  
  lastMotorUpdate = millis();
}

#include <ezLCDLib.h>

const bool DEBUG = false;

enum RotDir {
  RD_CW = LOW,
  RD_CCW = HIGH
};

ezLCD3 lcd;

void updatePwrMeters(char l, char r)
{
  lcd.wvalue(WID_LPWR, l);
  lcd.wvalue(WID_RPWR, r);
}

void stop(void)                    //Stop
{
  if(DEBUG)
    lcd.Debug("STOP\r\n");
  updatePwrMeters(0,0);  
  digitalWrite(EL,0);
  digitalWrite(ML,RD_CW);
  digitalWrite(ER,0);
  digitalWrite(MR,RD_CW);
  moveDirection = MD_STOPPED;
  runTime = 0;
}
void advance(char a,char b)          //Move forward
{
  if(DEBUG)
    lcd.Debug("Advance\r\n");
  updatePwrMeters(a,b);
  analogWrite (EL,a);      //PWM Speed Control
  digitalWrite(ML,RD_CW);
  analogWrite (ER,b);
  digitalWrite(MR,RD_CCW);
  moveDirection = MD_FORWARD;
}
void back_off (char a,char b)          //Move backward
{
  if(DEBUG)
    lcd.Debug("Back off\r\n");
  analogWrite (EL,a);
  digitalWrite(ML,RD_CCW);
  analogWrite (ER,b);
  digitalWrite(MR,RD_CW);
  moveDirection = MD_REVERSE;
}
void turn_L (char a,char b)             //Turn Left
{
  if(DEBUG)
    lcd.Debug("Turn left\r\n");
  analogWrite (EL,a);
  digitalWrite(ML,RD_CCW);
  analogWrite (ER,b);
  digitalWrite(MR,RD_CCW);
  moveDirection = MD_LEFT;
}
void turn_R (char a,char b)             //Turn Right
{
  if(DEBUG)
    lcd.Debug("Turn right\r\n");
  analogWrite (EL,a);
  digitalWrite(ML,RD_CW);
  analogWrite (ER,b);
  digitalWrite(MR,RD_CW);
  moveDirection = MD_RIGHT;
}

//void current_sense()                  // current sense and diagnosis
//{
//  int val1=digitalRead(2);
//  int val2=digitalRead(3);
//  if(val1==HIGH || val2==HIGH){
//    counter++;
//    if(counter==3){
//      counter=0;
//      lcd.Debug("Warning\r\n");
//    }
//  }
//}

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

void setup(void)
{
  lcd.begin( );
  lcd.cls( BLACK, WHITE );
  lcd.font("0");

  encoderInit();//Initialize the module

  int i;
  for(i=4;i<=7;i++)
    pinMode(i, OUTPUT);
 
  lcd.print("Motor\rController\rTest");
  lcd.fontw( 1, "sans24" );
  lcd.theme( 1, 9, 3, 4, 0, 0, 8, 8, 8, 1, 1 );
  lcd.theme( 2, 5, 20, 3, 3, 3, 4, 4, 4, 2, 1 );
  lcd.string( WID_LEFT, "Left" ); // stringId 1
  lcd.string( WID_RIGHT, "Right" ); // stringId 2
  lcd.string( WID_FORWARD, "Forward"); // stringId 3
  lcd.string( WID_REVERSE, "Reverse"); // stringId 4
  lcd.string( WID_STOP, "STOP");
  
  lcd.button( WID_LEFT, x1Pos, yPos, width, height, option, alignment, radius, 1, 1 );
  lcd.button( WID_RIGHT, x2Pos, yPos, width, height, option, alignment, radius, 1, 2 );
  lcd.button( WID_FORWARD, xPos, y1Pos, width, height, option, alignment, radius, 1, 3 );
  lcd.button( WID_REVERSE, xPos, y4Pos, width, height, option, alignment, radius, 1, 4 );
  lcd.button( WID_STOP, xPos, yPos, width, height, option, alignment, radius, 2, 5 );
  lcd.xy(xPos + width + 5, yPos + height + 10);
  lcd.print("Left: ");
  lcd.digitalMeter(WID_LTICKS, xPos + width + 50, yPos + height + 10, 70, 24, 14, 0, 5, 0, 1);
  lcd.xy(xPos + width + 5, yPos + height + 10 + 30);
  lcd.print("Right: ");
  lcd.digitalMeter(WID_RTICKS, xPos + width + 50, yPos + height + 10 + 30, 70, 24, 14, 0, 5, 0, 1);

  lcd.xy(0, yPos + height + 10);
  lcd.print("Left: ");
  lcd.digitalMeter(WID_LPWR, 50, yPos + height + 10, 50, 24, 14, 0, 3, 0, 1);
  lcd.xy(0, yPos + height + 10 + 30);
  lcd.print("Right: ");
  lcd.digitalMeter(WID_RPWR, 50, yPos + height + 10 + 30, 50, 24, 14, 0, 3, 0, 1);

  // Motor controller pins
  digitalWrite(EL,LOW);
  digitalWrite(ER,LOW);
  pinMode(2,INPUT);
  pinMode(3,INPUT);

  // Bumper pins
  pinMode(BMP_L, INPUT);
  pinMode(BMP_C, INPUT);
  pinMode(BMP_R, INPUT);
}

unsigned long printTime = 0;

void loop(void)
{
//  static unsigned long timePoint = 0;    // current sense and diagnosis,if you want to use this
//   if(millis() - timePoint > 1000){       //function,please show it & don't forget to connect the IS pins to Arduino
//   current_sense();
//   timePoint = millis();
//   }
  
  touch = lcd.wstack(0);
  
  if(touch >= WID_LEFT && touch <= WID_REVERSE) {
      runTime = millis();
      durationL = 0;
      durationR = 0;    
  }
  
  switch(touch)
  {
    case WID_LEFT:
      turn_L (L_POWER,R_POWER);
      break;
    case WID_RIGHT:
      turn_R (L_POWER,R_POWER);
      break;
    case WID_FORWARD:
      //advance (L_POWER,R_POWER);   //move forward
      motorEncoderTarget[MotorLeft] = PPM;
      motorPower[MotorLeft] = L_POWER;
      break;
    case WID_REVERSE:
      back_off (L_POWER,R_POWER);   //move back
      break;
    case WID_STOP:
      motorPower[MotorLeft] = 0;
      durationL = 0;
      durationR = 0;
      break;
  }

  if(moveDirection == MD_REVERSE) {
    if(abs(durationL) >= PPM && abs(durationR) >= PPM)
      motorPower[MotorLeft] = 0;
  } else if (moveDirection == MD_LEFT || moveDirection == MD_RIGHT) {
    if(abs(durationL) >= HALF_TURN && abs(durationR) >= HALF_TURN)
      motorPower[MotorLeft] = 0;
  }
  
  if(runTime != 0 && millis() - runTime > 10000) {
    motorPower[MotorLeft] = 0;
  }

  bool bmp_l_state_new = digitalRead(BMP_L);
  if(DEBUG && bmp_l_state_new == LOW && bmp_l_state == HIGH)
    lcd.Debug("LEFT Bumper triggered\r\n");
  bmp_l_state = bmp_l_state_new;

  bool bmp_c_state_new = digitalRead(BMP_C);
  if(DEBUG && bmp_c_state_new == LOW && bmp_c_state == HIGH)
    lcd.Debug("CENTER Bumper triggered\r\n");
  bmp_c_state = bmp_c_state_new;

  bool bmp_r_state_new = digitalRead(BMP_R);
  if(DEBUG && bmp_r_state_new == LOW && bmp_r_state == HIGH)
    lcd.Debug("RIGHT Bumper triggered\r\n");
  bmp_r_state = bmp_r_state_new;

  if(bmp_l_state == LOW || bmp_c_state == LOW || bmp_r_state == LOW) {
    back_off(L_POWER, R_POWER);
    delay(1000);
    motorPower[MotorLeft] = 0;
  }

  updateMotors();
    
  // encoder ticks
  if(millis() - printTime > 100)
  {
    lcd.wvalue(WID_LTICKS, durationL);
    lcd.wvalue(WID_RTICKS, durationR);
    printTime = millis();
  }
  delay(25);
}
