//
// Autonomous RC Car
// ARDUINO (MEGA) SETUP:
// =======================
// Adafruit GPS = 19(RX) u & 18(TX)
// 9DoF = I2C :  SCL (21) &  SDA (20)
// Motor Driver = _M1nSLEEP = 2; _M1nFAULT = 6; _M1DIR = 7; _M1PWM = 9; _M1CS = A0;
//                _M2nSLEEP = 4; _M2nFAULT = 12; _M2DIR = 8; _M2PWM = 10; _M2CS = A1;
// Ultrasonic HC-SR04 (trig,echo) (A6,A7) (A8,A9) (A10,A11)   
// Bluetooth HC-05 17(RX2) & 16(TX2)
// Wheel Encoders:  21(Left A) & 19(Right A) & 18 & 20

int count = -1;
int digit = 0;
double velocity = 0;
// #include <SoftReset.h>                          
//#include "DualG2HighPowerMotorShield.h"             // motor driver
// #include "quaternionFilters.h"                    //9DoF
// #include "MPU9250.h"                              //9DoF
//#include <SparkFunMPU9250-DMP.h>
//#include <waypointClass.h>                        // custom class to manage GPS waypoints
//#include <Adafruit_GPS.h>                         // GPS
#include <math.h>                                 // used by: GPS

//#include <Pixy2.h>                                //Pixy2 Camera
#include <PIDLoop.h>                              //Pixy2 Camera
#include <Encoder.h>
//Encoder
const int pinLeftA = 18;
const int pinLeftB = 19;
const int pinRightA = 20;
const int pinRightB = 21;

Encoder leftEnc(pinLeftA, pinLeftB);
Encoder rightEnc(pinRightA, pinRightB);
//process encoder values
const double leftDir = 1.0; const double rightDir = -1.0; //control direction of resultant speed
const double ticksPerRevolution = 48; //number of up and down from both channel A and B 12 * 2 * 2 = 48
const double gearRatio = 75.0; //gear ratio from wheel to shaft
const double wheelDiameter = 0.10; //diameter of wheels
unsigned long prevLeftTime  = 0; unsigned long prevRightTime = 0; //millis
long oldLeft = -999; long oldRight = -999;
long dispLeft = 0; long dispRight = 0;
long countLeft = 0; long countRight = 0;
double leftRevPSec = 0.0; double rightRevPSec= 0.0;
double leftSpeed = 0.0; double rightSpeed = 0.0;
const long counterThreshold = 100; //num of accumulated spins back and frontwards before update speed
const unsigned long timeOutThreshold = 500; //for super low speed or stop case no accumulation is done so go by timeout
const double milliMultiplier = 1000.0; //since divided by ms instead of seconds

//PID Speed Implementaion
double leftIntendedSpeedMs;
double rightIntendedSpeedMs;
const double maxInputAllowed = 70;
double leftKp = 50.0;
double leftKi = 0.0;
double leftKd = 0.0;
double rightKp = 50.0;
double rightKi = 0.0;
double rightKd = 0.0;
const double maxWindUp = 100.0;
double leftCurrentInput = 0.0; double rightCurrentInput = 0.0;
double leftPrevError = 0.0; double rightPrevError = 0.0;
unsigned long leftPrevPidTime = 0; unsigned long rightPrevPidTime = 0;
double leftAccmError = 0.0; double rightAccmError = 0.0;


//#define DEBUG YES                   // debug mode; uses Serial Port, displays diagnostic information, etc. 
//#define NO_GPS_WAIT YES           // define this for debugging to skip waiting for GPS fix

//Linefollowing
double FAST      =  0.21;
double SLOW      =  0.10;
#define X_CENTER    (pixy.frameWidth/2)

Pixy2 pixy;
PIDLoop headingLoop(170, 15, 10, false);
//Dual G2 High Power Motor Shield
DualG2HighPowerMotorShield18v18 md;
#define TURN_LEFT 2
#define TURN_RIGHT 3
#define TURN_STRAIGHT 1
const int Forward = 30;
int ForwardRight = 30;
const int Backward = -30;
bool STOP = false;

//ultrasonic sensors
int trigPin = 11;    // Trigger
int echoPin = 12;    // Echo
long duration, cm, inches;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// Steering/turning 
enum directions {left = TURN_LEFT, right = TURN_RIGHT, straight = TURN_STRAIGHT} ;
directions turnDirection = straight;

//move state
bool Linefollow = false;
bool GPSfollow = false;
unsigned long toMoveTimer = 0;
unsigned long toMoveTimeOut = 180000;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() 
{ 
  Serial.begin(115200); 

  //Ultrasonic sensors
  //Define inputs and outputs
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  ////////////////////////////////////

  Serial.print("Setting up Pixy");
  
  //Pixy2 Setup
  pixy.init();
  
  // Turn on both lamps, upper and lower for maximum exposure
  pixy.setLamp(1, 1);
  // change to the line_tracking program.  Note, changeProg can use partial strings, so for example,
  // you can change to the line_tracking program by calling changeProg("line") instead of the whole
  // string changeProg("line_tracking")
  pixy.changeProg("line");

  Serial.print("Setting up MD");
  
  // setupSn();
  // Start motor drivers
  md.init();
  md.calibrateCurrentOffsets();
  md.flipM1(true);
  md.flipM2(true);

  //Init Encoder stuff
  oldLeft = leftEnc.read();
  oldRight = rightEnc.read();
  

  delay(1000);

  Serial.print("Setup Complete");
  
} 

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop()
{
 
  //Ultrasonic sensors
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
 
  // Convert the time into a distance
  cm = (duration/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343

  Serial.print(cm);
  Serial.print("cm");
  Serial.println();

///////////////////////////////////////////
  if ((!Linefollow)){
    rightIntendedSpeedMs = 0;
    leftIntendedSpeedMs = 0;
    md.setM1Speed(0);
    md.setM2Speed(0);
    Linefollow = true;
  }
  EncoderChecks();
  
  if ((Linefollow)){
    Serial.println("Line MODE");
    LINEFOLLOWING_MODE();
  }

  
  delay(50);
  
}  // loop()

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void EncoderChecks(){
  unsigned long timeNow = millis();
  long currentLeft = leftEnc.read();
  long deltaLeft = currentLeft - oldLeft;
  dispLeft += deltaLeft;
  countLeft += abs(deltaLeft);
  oldLeft = currentLeft;

  unsigned long leftDeltaTime = timeNow - prevLeftTime;
  if ((countLeft > counterThreshold) || (leftDeltaTime > timeOutThreshold)) {
    leftRevPSec = calRevPSec(dispLeft, leftDeltaTime, leftDir);
    leftSpeed = calSpeedFromRevPerSec(leftRevPSec, wheelDiameter);
    dispLeft = 0;
    countLeft = 0;
    prevLeftTime = timeNow;
  }

  timeNow = millis();
  long currentRight = rightEnc.read();
  long deltaRight = currentRight - oldRight;
  dispRight += deltaRight;
  countRight += abs(deltaRight);
  oldRight = currentRight;
  
  unsigned long rightDeltaTime = timeNow - prevRightTime;
  if ((countRight > counterThreshold) || (rightDeltaTime > timeOutThreshold)) {
    rightRevPSec = calRevPSec(dispRight, rightDeltaTime, rightDir);
    rightSpeed = calSpeedFromRevPerSec(rightRevPSec, wheelDiameter);
    dispRight = 0;
    countRight = 0;
    prevRightTime = timeNow;
  }

  Serial.print("\nLeft Encoder Current: "); Serial.print(currentLeft);
  Serial.print("\nRight Encoder Current: "); Serial.println(currentRight);

  //Compute PID
  
  double leftCurrentError = -leftSpeed + leftIntendedSpeedMs;
  double currentPidTime = millis();
  double leftDelTime = double(currentPidTime - leftPrevPidTime);
  leftAccmError += 0.5 * (leftCurrentError + leftPrevError) * leftDelTime;
  if (leftAccmError > maxWindUp)
  {
    leftAccmError = maxWindUp;
  }
  else if (leftAccmError < -maxWindUp)
  {
    leftAccmError = -maxWindUp;
  }
  double leftRocError = (leftCurrentError - leftPrevError) / leftDelTime;
  leftCurrentInput -= leftCurrentError * leftKp + leftAccmError * leftKi + leftRocError * leftKd;
  if (leftCurrentInput > maxInputAllowed)
  {
    leftCurrentInput = maxInputAllowed;
  }
  else if (leftCurrentInput < -maxInputAllowed)
  {
    leftCurrentInput = -maxInputAllowed;
  }
 // leftCurrentInput = min(0.0, leftCurrentInput);
  if ((leftIntendedSpeedMs)==0 ||(STOP)){
    leftCurrentInput = 0;
  }
  Serial.print("leftCurrentInput");    
  md.setM1Speed(leftCurrentInput);
  leftPrevPidTime = currentPidTime;
  leftPrevError = leftCurrentError;

  double rightCurrentError = -rightSpeed + rightIntendedSpeedMs;
  currentPidTime = millis();
  double rightDelTime = double(currentPidTime - rightPrevPidTime);
  rightAccmError += 0.5 * (rightCurrentError + rightPrevError) * rightDelTime;
  if (rightAccmError > maxWindUp)
  {
    rightAccmError = maxWindUp;
  }
  else if (rightAccmError < -maxWindUp)
  {
    rightAccmError = -maxWindUp;
  }
  double rightRocError = (rightCurrentError - rightPrevError) / rightDelTime;
  rightCurrentInput -= rightCurrentError * rightKp + rightAccmError * rightKi + rightRocError * rightKd;
  if (rightCurrentInput > maxInputAllowed)
  {
    rightCurrentInput = maxInputAllowed;
  }
  else if (rightCurrentInput < -maxInputAllowed)
  {
    rightCurrentInput = -maxInputAllowed;
  }
   rightCurrentInput = min(0.0, rightCurrentInput);
 
  if ((rightIntendedSpeedMs==0 ||(STOP))){
    Serial.print("\nSTOOOOOOOP");
    rightCurrentInput = 0;
  }
  Serial.print(rightCurrentInput);
  md.setM2Speed(rightCurrentInput);
  rightPrevPidTime = currentPidTime;
  rightPrevError = rightCurrentError;

  Serial.print("\nLeft Current Speed: "); Serial.print(leftSpeed);
  Serial.print("\tLeft Intended Speed: "); Serial.println(leftIntendedSpeedMs);
  Serial.print("\nRight Current Speed: "); Serial.print(rightSpeed);
  Serial.print("\tRight Intended Speed: "); Serial.println(rightIntendedSpeedMs);
}//void EncoderChecks()

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double calRevPSec(long inpDisplacement, unsigned long inpMillisTimeDiff, double inpDir)
{
  return double(inpDisplacement) / double(inpMillisTimeDiff) / double(ticksPerRevolution * gearRatio) * inpDir * milliMultiplier;
}

double calSpeedFromRevPerSec(double inpRevPSec, double inpWheelDiameter)
{
  return inpRevPSec * inpWheelDiameter * PI;  
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void LINEFOLLOWING_MODE()
{
  int8_t res;
  int32_t error; 
  float left, right;
  char buf[96];

  // Get latest data from Pixy, including main vector, new intersections and new barcodes.
  res = pixy.line.getMainFeatures();

  // If error or obstacle detected, stop motors
  if ((res<=0) || (cm < 30))
  { 
    STOP = true;
    md.setM2Speed(0);
    md.setM1Speed(0);
    leftIntendedSpeedMs = 0;
    rightIntendedSpeedMs = 0;
    Serial.println("STOP");
  }
  else{
    STOP = false;
  }


    // We found the vector...
  if ((res&LINE_VECTOR) && !STOP)
  {
    // Calculate heading error with respect to m_x1, which is the far-end of the vector,
    // the part of the vector we're heading toward.
    error = (int32_t)pixy.line.vectors->m_x1 - (int32_t)X_CENTER;

    pixy.line.vectors->print();

    // Perform PID calcs on heading error.
    headingLoop.update(error);

    // separate heading into left and right wheel velocities. /167
    left = headingLoop.m_command/300.0;
    right = -headingLoop.m_command/300.0;

    // If vector is heading away from us (arrow pointing up), things are normal.
    if (pixy.line.vectors->m_y0 > pixy.line.vectors->m_y1)
    {
     
        left += FAST;
        Serial.print("\n");Serial.print(left);
        right += FAST;
        Serial.print("\n");Serial.print(right);
       
          
    }

    if (left > 0.3)
    {
      left == 0.3;
    }
        if (right > 0.3)
    {
      right == 0.3;
    }
        if (left < -0.3)
    {
      left == -0.3;
    }
        if (right < -0.3)
    {
      right == -0.3;
    }
    leftIntendedSpeedMs = left;
    rightIntendedSpeedMs = right;
  //  Serial.print('\n'); Serial.print(leftIntendedSpeedMs); Serial.print(rightIntendedSpeedMs);
//    processGPS();
  }
  else if (!(res&LINE_VECTOR)){
    leftIntendedSpeedMs = 0.0;
    rightIntendedSpeedMs = 0.0;
    Serial.print("NO LINE FOUND");
  }
}//void LINEFOLLOWING_MODE()

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// old code with constant speed differences

void Move(void)
{ 
  if ((cm < 30))
  {
    STOP = true;
    md.setM1Speed(0);
    md.setM2Speed(0);
    Serial.print ("STOP");
  }
  else
  {
   STOP = false; 
   }
  if ((turnDirection == straight) && !STOP)
  {
    md.setM1Speed(Forward);
    md.setM2Speed(ForwardRight);
    if (ForwardRight > 50){
      ForwardRight = 50;
    }
    Serial.print("STRAIGHT");
  }   
  if((turnDirection == left) && !STOP)
 {
    md.setM1Speed(Backward);
    md.setM2Speed(Forward);
    Serial.print("LEFT");
 }
  if((turnDirection == right) && !STOP)
 {
    md.setM1Speed(Forward);
    md.setM2Speed(Backward);
    Serial.print("RIGHT");
 }
}   // Move()
