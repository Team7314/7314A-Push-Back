/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       student                                                   */
/*    Created:      7/18/2025, 5:09:05 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/


#include "vex.h"


using namespace vex;


// A global instance of competition
competition Competition;
brain Brain;
controller Controller1;
inertial gyroT = inertial(PORT10);
motor LF(PORT1, ratio18_1, false);
motor RF(PORT2, ratio18_1, true);
motor LB(PORT3, ratio18_1, false);
motor RB(PORT4, ratio18_1, true);
motor IR(PORT5, ratio6_1, true);
motor IL(PORT6, ratio6_1, false);
motor IR2(PORT7, ratio6_1, true);
motor IL2(PORT8, ratio6_1, false);
digital_out Deloader = digital_out(Brain.ThreeWirePort.A);
digital_out Descorer = digital_out(Brain.ThreeWirePort.B);
optical THESENSOR (PORT11);
// functions or something i guess \_[-_-]_/
int value =  THESENSOR.hue();
const int RED_VAL = 20;
const int BLUE_VAL1 = 120;
const int BLUE_VAL2 = 230;
const int MOTOR_SPEED = 80;
const int SPIN_CLOCKWISE = -1 * MOTOR_SPEED;
const int SPIN_COUNTER_CLOCKWISE = MOTOR_SPEED;

void colorsensor(bool vexc) {
  const bool FOUND_BLUE = THESENSOR.hue() >= BLUE_VAL1 && THESENSOR.hue() <= BLUE_VAL2;
  const bool FOUND_RED = THESENSOR.hue() <= RED_VAL;
  /*if(FOUND_RED){
    //the color is red
     wait (500, msec);
    IL2.spin(forward, SPIN_CLOCKWISE, pct); //call the color sorting function 
    
    // Keep
  }*/
 if (vexc) {
  if (FOUND_BLUE){
    IL2.spin(forward, 55, pct);
    wait(150, msec);
    // Eject
  }
  else if (FOUND_RED) {
    IL2.spin(reverse, 80,  pct);
    wait(150, msec);
  }
  else { //the color is neither red nor blue
    IL2.stop();}
}
}
//functions or something i guess
double YOFFSET = 50; //offset for the display
//Writes a line for the diagnostics of a motor on the Brain


void MotorDisplay(double y, double curr, double temp)
{
 Brain.Screen.setFillColor(transparent);
 Brain.Screen.printAt(5, YOFFSET + y, "Current: %.1fA", curr);
  if (curr < 1){
   Brain.Screen.setFillColor(green);
 } else if(curr >= 1 && curr  <= 2.5) {
   Brain.Screen.setFillColor(yellow);
 } else {
   Brain.Screen.setFillColor(red);
 }
 Brain.Screen.drawRectangle(140, YOFFSET + y - 15, 15, 15);


  Brain.Screen.setFillColor(transparent);
 Brain.Screen.printAt(160, YOFFSET + y, "Temp: %.1fC", temp);
  if (temp < 45){
   Brain.Screen.setFillColor(green);
 } else if(temp <= 50 && temp  >= 45){
   // TRUE and TRUE --> True
   // TRUE and FALSE --> False
   // FALSE and FALSE --> False
   Brain.Screen.setFillColor(yellow);
 }else if(temp >= 60){
   // TRUE and TRUE --> True
   // TRUE and FALSE --> False
   // FALSE and FALSE --> False
   Brain.Screen.setFillColor(blue);
 } else {
   Brain.Screen.setFillColor(red);
 }
 Brain.Screen.drawRectangle(275, YOFFSET + y - 15, 15, 15);
 Brain.Screen.setFillColor(transparent);
}


//Displays information on the brain
void Display()
{
 double leftFrontCurr = LF.current(amp);
 double leftFrontTemp = LF.temperature(celsius);
 double leftBackCurr = LB.current(amp);
 double leftBackTemp = LB.temperature(celsius);
 double rightFrontCurr = RF.current(amp);
 double rightFrontTemp = RF.temperature(celsius);
 double rightBackCurr = RB.current(amp);
 double rightBackTemp = RB.temperature(celsius);
 double IRCurr = IR.current(amp);
 double IRTemp = IR.temperature(celsius);
 double ILCurr = IL.current(amp);
 double ILTemp = IL.temperature(celsius);
 double IR2Curr = IR2.current(amp);
 double IR2Temp = IR2.temperature(celsius);
 double IL2Curr = IL2.current(amp);
 double IL2Temp = IL2.temperature(celsius);


 if (LF.installed()) {
   MotorDisplay(1, leftFrontCurr, leftFrontTemp);
   Brain.Screen.printAt(300, YOFFSET + 1, "LeftFront");
 } else {
   Brain.Screen.printAt(5, YOFFSET + 1, "LeftFront Problem");
 }
 
 if (LB.installed()){
   MotorDisplay(31, leftBackCurr, leftBackTemp);
   Brain.Screen.printAt(300, YOFFSET + 31, "LeftBack");
 } else {
   Brain.Screen.printAt(5, YOFFSET + 31, "LeftBack Problem");
 }




 if (RF.installed()) {
   MotorDisplay(61, rightFrontCurr, rightFrontTemp);
   Brain.Screen.printAt(300, YOFFSET + 61, "RightFront");
 } else {
   Brain.Screen.printAt(5, YOFFSET + 61, "RightFront Problem");
 }
 
 if (RB.installed()) {
   MotorDisplay(91, rightBackCurr, rightBackTemp);
   Brain.Screen.printAt(300, YOFFSET + 91, "RightBack");
 } else {
   Brain.Screen.printAt(5, YOFFSET + 91, "RightBack Problem");
 }


if (IR.installed()) {
MotorDisplay(121, IRCurr, IRTemp);
Brain.Screen.printAt(300, YOFFSET + 121, "IR");
}else{
 Brain.Screen.printAt(5, YOFFSET + 121, "IR Problem");
}




if (IL.installed()) {
MotorDisplay(151, ILCurr, ILTemp);
Brain.Screen.printAt(300, YOFFSET + 151, "IL");
}else{
 Brain.Screen.printAt(5, YOFFSET + 151, "IL Problem");
}




if(IR2.installed()) {
MotorDisplay(181, IR2Curr, IR2Temp);
Brain.Screen.printAt(300, YOFFSET + 181, "IR2");
}else{
 Brain.Screen.printAt(5, YOFFSET + 181, "IR2 Problem");
}




if(IL2.installed()) {
 MotorDisplay(211, IL2Curr, IL2Temp);
Brain.Screen.printAt(300, YOFFSET + 211, "IL2");
}else{
 Brain.Screen.printAt(5, YOFFSET + 211, "Il2 Problem");
}
}


void driveTank(int Lspeed,int Rspeed, int wt){
 LF.spin(forward, Lspeed, pct);
 RF.spin(forward, Rspeed, pct);
 LB.spin(forward, Lspeed, pct);
 RB.spin(forward, Rspeed, pct);
 wait (wt, msec);
}


void leftturn(int Lspeed, int Rspeed, int wt){
 //Turn for certain time spans
 LF.spin(reverse, Lspeed, pct);
 RF.spin(forward, Rspeed, pct);
 LB.spin(reverse, Lspeed, pct);
 RB.spin(forward, Rspeed, pct);
 wait (wt, msec);
}


void Xdrive(int LFspeed, int RFspeed, int LBspeed, int RBspeed, int wt){
 LF.spin(forward, LFspeed, pct);
 RF.spin(forward, RFspeed, pct);
 LB.spin(forward, LBspeed, pct);
 RB.spin(forward, RBspeed, pct);
 wait(wt, msec);
}




void drivebrake(){
 LF.stop(brake);
 RF.stop(brake);
 LB.stop(brake);
 RB.stop(brake);
}


void g_print(){


}


void gyroturn(float target)
{
 float heading = 0.0; //initialize a variable for heading, note these are copies of yunzes work, are temporary
 float accuracy = 2.0; //how accurate to make the turn in degrees
 float error = target-heading;
 float kp = 1.2;                                // kp original value: 2.0
 float speed = kp * error;
 gyroT.setRotation(0.0, deg); // resets gyro to 0 degrees


 while(fabs(error) >= accuracy) // fabs(error) = float absolute value(error)
 {
   speed=kp*error;
   driveTank(speed, -speed, 10);
   heading = gyroT.rotation(deg);
   error = target - heading;
   Brain.Screen.printAt(10,30, "error= %0.2f", error);
 }
 drivebrake();
 //driveTank(0, 0, 0);
 /*drivebrake();
 heading = gyroT.rotation(degrees);
 Brain.Screen.printAt(10, 30, "heading= %0.2f", heading);
 Brain.Screen.printAt(240, 120, "HERE");*/
}


void inchdrive(float target)
{
 float x = 0.0;
 float accuracy = 1.0;
 float error = target - x;
 float kp = 7.06;                         // kp original value: 7.06
 float speed = kp * error;
 LF.setPosition(0, rev);


 while (fabs(error) >= accuracy)
 {
   speed = kp * error;
   driveTank(speed, speed, 10);
   x = LF.position(rev) * 3.25 * 3.14 * 0.7;
   error = target - x;
 }
 drivebrake();
}
void sidedrive(float target)
{
 float y = 0.0;
 float accuracy = 1.0;
 float error = target - y;
 float kp = 7.06;
 float speed = kp * error;
 LF.setPosition(0, rev);


 while (fabs(error) >= accuracy)
 {
   speed = kp * error;
   Xdrive(speed, -speed, -speed, speed, 10);
   y = LF.position(rev) * 3.25 * 3.14 * 0.7;
   error = target - y;
 }
 drivebrake();
}




void Intake(int Ispeed, int wt) {
IR.spin(reverse, Ispeed, pct);
  IL.stop(brake);
  IR2.spin(reverse, 100, pct);
  IL2.spin(reverse, Ispeed, pct);
}
void colorintake(int Ispeed) {
  IR.spin(reverse, Ispeed, pct);
  IL.stop(brake);
  IR2.spin(reverse, 100, pct);
  colorsensor(true);
}
void Bottomscore(int Ispeed, int wt) {
 IL.spin(forward, Ispeed, pct);
 IR.spin(forward, Ispeed, pct);
}


void Middlescore(int Ispeed, int wt) {
IL.spin(forward, Ispeed, pct);
     IR.spin(reverse, 90, pct);
     IR2.spin(forward, 75, pct);
}


void Topscore(int Ispeed, int wt) {
    IL.spin(forward, Ispeed, pct);
     IR.spin(reverse, Ispeed, pct);
     IL2.spin(forward, 70, pct);
     IR2.spin(reverse, 75, pct);
}


void Ibrake(){
 IL.stop(brake);
 IR.stop(brake);
 IR2.stop(brake);
 IL2.stop(brake);
}


void Armup(){
 Deloader.set(false);
}
void Armdown(){
 Deloader.set(true);
}
void Dejam(int time){
   IL.spin(forward, 80, pct);
   wait (time, msec);
   IL.spin(reverse, 80, pct);
   wait (time, msec);

}

// define your global instances of motors and other devices here


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function. You must return from this function    */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/


void pre_auton(void) {


 // All activities that occur before the competition starts
 // Example: clearing encoders, setting servo positions, ...


 gyroT.calibrate();
 while (gyroT.isCalibrating());
 wait (50, msec);
 Deloader.set(false);
 }


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

// ----- SIMPLE CONSTANTS STUDENTS CAN TUNE -----
const double WHEEL_DIAM = 3.25;         // wheel size in inches
const double PI_VAL     = 3.14159;
const double GYRO_KP    = 2.0;          // heading correction gain
const double DEGREES_PER_INCH = 27.0;  // wheel circumference

// -----------------------------------------------
// --------------- AUTON PARAMS ------------------
// -----------------------------------------------
  const int FORWARD_SPEED = 18;
  const int TURN_SPEED = 20;
  const int SCORING_SPEED = 45;

double degreesPerInch() {
  return DEGREES_PER_INCH;
}

// Drive straight using all 4 motors + gyro heading correction
void driveForwardInches(double inches, int speedPct) {
  // reset encoders
  LF.resetPosition();
  RF.resetPosition();
  LB.resetPosition();
  RB.resetPosition();

  double targetDeg = inches * degreesPerInch();

  // starting heading
  double startAngle = gyroT.heading();

  while (true) {
    // average of all 4 wheels
    double avgPos = (LF.position(degrees) +
                     RF.position(degrees) +
                     LB.position(degrees) +
                     RB.position(degrees)) / 4.0;

    if (fabs(avgPos) >= fabs(targetDeg)) {
      break;
    }

    // gyro correction
    double currentAngle = gyroT.heading();
    double error = startAngle - currentAngle;

    if (error > 180)  error -= 360;
    if (error < -180) error += 360;

    double correction = error * GYRO_KP;

    double leftPower  = speedPct + correction;
    double rightPower = speedPct - correction;

    // clamp power to [-100, 100]
    if (leftPower  > 100) leftPower  = 100;
    if (leftPower  < -100) leftPower = -100;
    if (rightPower > 100) rightPower = 100;
    if (rightPower < -100) rightPower = -100;

    // tank-style straight drive
    LF.spin(forward, leftPower,  pct);
    LB.spin(forward, leftPower,  pct);
    RF.spin(forward, rightPower, pct);
    RB.spin(forward, rightPower, pct);

    wait(20, msec);
  }

  drivebrake();
}

void backToWallSlow() {
  LF.spin(reverse, 20, pct);
  LB.spin(reverse, 20, pct);
  RF.spin(reverse, 20, pct);
  RB.spin(reverse, 20, pct);
  wait(700, msec);    // drive slowly back ~0.7s
  drivebrake();
}

// Turn to absolute angle using gyroT and your existing driveTank
void turnToAngle(double targetAngle, int baseSpeedPct) {
  double accuracy = 2.0;   // degrees
  double kP       = 1.2;   // simple P

  while (true) {
    double current = gyroT.rotation(deg);
    double error   = targetAngle - current;

    if (error > 180)  error -= 360;
    if (error < -180) error += 360;

    if (fabs(error) < accuracy) {
      break;
    }

    double speed = kP * error;

    // limit speed
    if (speed >  baseSpeedPct)  speed =  baseSpeedPct;
    if (speed < -baseSpeedPct)  speed = -baseSpeedPct;

    driveTank(speed, -speed, 10);
  }

  drivebrake();
}

// X-drive strafe using LF/LB/RF/RB + gyro correction to keep heading
void strafeRightInches(double inches, int speedPct) {
  // reset encoders
  LF.resetPosition();
  RF.resetPosition();
  LB.resetPosition();
  RB.resetPosition();

  double targetDeg = inches * degreesPerInch();

  double startAngle = gyroT.heading();

  while (true) {
    // for strafing, use LF and RB (they move same direction)
    double avgPos = (LF.position(degrees) + RB.position(degrees)) / 2.0;

    if (fabs(avgPos) >= fabs(targetDeg)) {
      break;
    }

    double currentAngle = gyroT.heading();
    double error = startAngle - currentAngle;
    if (error > 180)  error -= 360;
    if (error < -180) error += 360;

    double correction = error * GYRO_KP;

    double flPower =  speedPct + correction;
    double frPower = -speedPct - correction;
    double lbPower = -speedPct + correction;
    double rbPower =  speedPct - correction;

    // clamp
    auto clamp = [](double v) {
      if (v > 100) return 100.0;
      if (v < -100) return -100.0;
      return v;
    };

    flPower = clamp(flPower);
    frPower = clamp(frPower);
    lbPower = clamp(lbPower);
    rbPower = clamp(rbPower);

    // X-drive strafe pattern
    LF.spin(forward, flPower, pct);
    RF.spin(forward, frPower, pct);
    LB.spin(forward, lbPower, pct);
    RB.spin(forward, rbPower, pct);

    wait(20, msec);
  }

  drivebrake();
}


void firstScoringCycle() {
// First scoring cycle
  ////////////////////////////////////////////////////////////////////////
 driveForwardInches(12, FORWARD_SPEED);   // go forward 12"
  wait(300, msec);
  turnToAngle(-45, TURN_SPEED);         // turn right 90 degrees
  wait(300, msec);
  Intake(80, 0);
  driveForwardInches(17.5, FORWARD_SPEED);   // go forward 15.5"
  wait(300, msec);
  driveForwardInches(4, -FORWARD_SPEED);   // go backward 15.5"
  turnToAngle(90, TURN_SPEED);     
  driveForwardInches(26, FORWARD_SPEED * 2);    // go forward 52"
  turnToAngle(45, TURN_SPEED);
  strafeRightInches(8, FORWARD_SPEED);    // strafe right 5"
  driveForwardInches(25, FORWARD_SPEED);   // go forward 17"
  driveForwardInches(5, -FORWARD_SPEED);   // go backward 3"
  strafeRightInches(28, FORWARD_SPEED);    // strafe right 7"
  turnToAngle(0, TURN_SPEED);
  strafeRightInches(0.25, FORWARD_SPEED);    // strafe right 2"
  driveForwardInches(10, FORWARD_SPEED);   // go forward 10"
 Topscore(SCORING_SPEED * 2, 0);    // score top
  wait(3, sec);
  Dejam(100);
  wait(1, sec);
  Topscore(SCORING_SPEED * 2, 0);
  wait(3, sec);
  Dejam(100);
  wait(1, sec);
  Topscore(SCORING_SPEED * 2, 0);
  wait(3, sec);
  Dejam(100);
  wait(1, sec);
  Topscore(SCORING_SPEED * 2, 0);
  wait(3, sec);
  Ibrake();
    ////////////////////////////////////////////////////////////////////////
}

void secondScoringCycle() {
    // Deloading cycle
  ////////////////////////////////////////////////////////////////////////
  driveForwardInches(5, -FORWARD_SPEED);   // go backward 5"
  turnToAngle(180, TURN_SPEED*2);
  Armdown();
  Intake(80, 0);
  wait(150, msec);
  driveForwardInches(12, 40);   // go forward 18"
  wait(100, msec);
  driveForwardInches(12, 100);   // go backward 2
  wait(300, msec);
  driveForwardInches(2, -FORWARD_SPEED);   // go backward 2"
  driveForwardInches(2, FORWARD_SPEED);   // go forward 2"
  wait(100, msec);
driveForwardInches(2, -FORWARD_SPEED);   // go backward 2"
  driveForwardInches(2, FORWARD_SPEED);   // go forward 2"
  wait(100, msec);
  driveForwardInches(2, -FORWARD_SPEED);   // go backward 2"
  driveForwardInches(2, FORWARD_SPEED);   // go forward 2"
  wait(100, msec);
  driveForwardInches(2, -FORWARD_SPEED);   // go backward 2"
  driveForwardInches(2, FORWARD_SPEED);   // go forward 2"
  wait(1.5, sec);
  driveForwardInches(5, -FORWARD_SPEED);   // go backward 5"
  Armup();
  turnToAngle(0, TURN_SPEED);
  driveForwardInches(18, FORWARD_SPEED);   // go forward 20"
  strafeRightInches(2, FORWARD_SPEED);    // strafe right 2"
  driveForwardInches(4, FORWARD_SPEED);   // go forward 2"
  Topscore(SCORING_SPEED * 2, 0);    // score top
  wait(3, sec);
  Dejam(100);
  wait(500, msec);
  Topscore(SCORING_SPEED * 2, 0);
  wait(3, sec);
  Dejam(100);
  wait(500, msec);
  Topscore(SCORING_SPEED * 2, 0);
  wait(3, sec);
  Ibrake();
}

void incrementalForwardSpeed(int targetSpeed, int acceleration_step, int speed_step) {
  for(int speed = 0; speed <= targetSpeed; speed += speed_step) {
    Xdrive(speed, speed, speed, speed, acceleration_step);
  }
}

void autonParking() {
  driveForwardInches(10, -FORWARD_SPEED);   // go backward 10"
  turnToAngle(-90, TURN_SPEED);
  driveForwardInches(16, FORWARD_SPEED);   // go forward 16"
  strafeRightInches(21, -FORWARD_SPEED);    // strafe right 25"

  // Accelerated forward to park
  const int MAX_SUPER_SPEED = 200;
  const int SPEED_STEP = 5;
  const int ACCELERATION_STEP = 2; // milliseconds between speed increments
  incrementalForwardSpeed(MAX_SUPER_SPEED, ACCELERATION_STEP, SPEED_STEP);

}

void autonParking2() {
 driveForwardInches(10, -FORWARD_SPEED * 2);   // go backward 10"
  strafeRightInches(28, -FORWARD_SPEED * 2);    // strafe left 14"
  turnToAngle(-135, TURN_SPEED);
  Intake(80, 0);
  wait(300, msec);

    // Accelerated forward to park
  const int MAX_SUPER_SPEED = 200;
  const int SPEED_STEP = 5;
  const int ACCELERATION_STEP = 2; // milliseconds between speed increments
  incrementalForwardSpeed(MAX_SUPER_SPEED, ACCELERATION_STEP, SPEED_STEP);

  // driveForwardInches(20, -FORWARD_SPEED);   // go backward 20"
  // Xdrive(200, 200, 200, 200, 100); //go forward FAST
  // Intake(80, 0);
  // wait(10, sec);
  // drivebrake();
}


void autonomous(void) {
  
  firstScoringCycle();
   secondScoringCycle();
  autonParking2();
 }


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/


void usercontrol(void) {
  Brain.resetTimer();
  THESENSOR.setLight(ledState::on);
  int Ispeed = 80;
  bool vexc = false;
 // User control code here, inside the loop
 //int T = 0;
 while (1) {
  colorsensor(vexc);
   Display();
   wait (1, msec);
   if( Controller1.ButtonA.pressing()) {         //Intake       
      colorintake(Ispeed);
      vexc = true;  
    }
   else if( Controller1.ButtonB.pressing()) {    //Bottom Score
      IL.spin(reverse, Ispeed, pct);
      IR.spin(forward, Ispeed, pct);
      wait(100, msec);
      IL.spin(forward, Ispeed, pct);
      IR.spin(forward, Ispeed, pct);
      vexc = false;
   }
   else if( Controller1.ButtonY.pressing()) {    //middle Score
     IL.spin(reverse, Ispeed, pct);
     IR.spin(reverse, Ispeed, pct);
     IR2.spin(forward, 75, pct);
     wait(100, msec);
     IL.spin(forward, Ispeed, pct);
     IR.spin(reverse, Ispeed, pct);
     IR2.spin(forward, 75, pct);
     vexc = false;
   }
   else if(Controller1.ButtonX.pressing()) {      //Top score 
     IL.spin(reverse, Ispeed, pct);
     IR.spin(reverse, Ispeed, pct);
     IL2.spin(forward, 70, pct);
     IR2.spin(reverse, 75, pct);
     wait(100, msec);
     IL.spin(forward, Ispeed, pct);
     IR.spin(reverse, Ispeed, pct);
     IL2.spin(forward, 70, pct);
     IR2.spin(reverse, 75, pct);
     vexc = false;
   }
  
   else if(Controller1.ButtonLeft.pressing()) {
     IL.stop(brake);
     IR.stop(brake);
     IR2.stop(brake);
     IL2.stop(brake);
   }
   else if(Controller1.ButtonR1.pressing()) {
    Descorer.set(true);
   }
   else if(Controller1.ButtonR2.pressing()) {
    Descorer.set(false);
   }
   else if(Controller1.ButtonL2.pressing()) {
     Deloader.set(true);
   }
   else if(Controller1.ButtonL1.pressing()){
     Deloader.set(false);
   }
   else if(Controller1.ButtonLeft.pressing()) {
    IL.stop(brake);
     IR.stop(brake);
     IR2.stop(brake);
     IL2.stop(brake);
   }

   int Y = Controller1.Axis3.position(); // Forward/Backward
   int X = Controller1.Axis4.position(); // Left and Right
   int R = Controller1.Axis1.position(); // Rotational


   //Xdrive(Y + X + R,Y - X + R,Y - X - R,Y + X - R, 10);
  
   LF.spin(forward, Y + X + R, percent);
   LB.spin(forward, Y - X + R, percent);
   RF.spin(forward, Y - X - R, percent);
   RB.spin(forward, Y + X - R, percent);






   // This is the main execution loop for the user control program.
   // Each time through the loop your program should update motor + servo
   // values based on feedback from the joysticks.


   // ........................................................................
   // Insert user code here. This is where you use the joystick values to
   // update your motors, etc.
   // ........................................................................
   //T = T + 20;
   wait(20, msec); // Sleep the task for a short amount of time to
                   // prevent wasted resources.
 }
}


//
// Main will set up the competition functions and callbacks.
//
int main() {
 // Set up callbacks for autonomous and driver control periods.
 Competition.autonomous(autonomous);
 Competition.drivercontrol(usercontrol);


 // Run the pre-autonomous function.
 pre_auton();


 // Prevent main from exiting with an infinite loop.
 while (true) {
   wait(100, msec);
 }
}

