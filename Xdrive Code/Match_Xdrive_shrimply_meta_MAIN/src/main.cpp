/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Team 7314                                                 */
/*    Created:      7/18/2025, 5:09:05 PM                                     */
/*    Description:  Unified Match Play Code - All Autonomous Routines         */
/*                                                                            */
/*    CONSOLIDATED FROM:                                                     */
/*    - LeftsideBlue_Xdrive_shrimply_meta_MAIN                               */
/*    - RedLeftside_Xdrive_shrimply_meta_MAIN                                */
/*    - RedRightside_Xdrive_simply_meta_main                                 */
/*    - RightsideBlue_Xdrive_shrimply_meta_MAIN                              */
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
optical THESENSOR = optical (PORT11);

int value =  THESENSOR.hue();
const int RED_VAL = 20;
const int BLUE_VAL1 = 120;
const int BLUE_VAL2 = 230;
const int MOTOR_SPEED = 80;
const int SPIN_CLOCKWISE = -1 * MOTOR_SPEED;
const int SPIN_COUNTER_CLOCKWISE = MOTOR_SPEED;

// Enums define named options instead of using 0, 1, 2, etc.
enum Color { RED, BLUE };
enum Side { RIGHT, LEFT };
enum Goal { CENTER, LONG };

// Global variable to store selected alliance color from auton selector
Color selectedColor;

// Color sensor logic for BLUE alliance
void colorsensorBlue(bool vexc) {
  const bool FOUND_BLUE = THESENSOR.hue() >= BLUE_VAL1 && THESENSOR.hue() <= BLUE_VAL2;
  const bool FOUND_RED = THESENSOR.hue() <= RED_VAL;
  
  if (vexc) {
    if (FOUND_RED) {
      IL2.spin(forward, 55, pct);
      wait(150, msec);
    }
    else if (FOUND_BLUE) {
      IL2.spin(reverse, 80, pct);
      wait(150, msec);
    }
    else {
      IL2.stop();
    }
  }
}

// Color sensor logic for RED alliance
void colorsensorRed(bool vexc) {
  const bool FOUND_BLUE = THESENSOR.hue() >= BLUE_VAL1 && THESENSOR.hue() <= BLUE_VAL2;
  const bool FOUND_RED = THESENSOR.hue() <= RED_VAL;
  
  if (vexc) {
    if (FOUND_BLUE) {
      IL2.spin(forward, 55, pct);
      wait(150, msec);
    }
    else if (FOUND_RED) {
      IL2.spin(reverse, 80, pct);
      wait(150, msec);
    }
    else {
      IL2.stop();
    }
  }
}

// Wrapper function that calls the appropriate alliance color sensor
void colorsensor(bool vexc) {
  if (selectedColor == BLUE) {
    colorsensorBlue(vexc);
  } else {
    colorsensorRed(vexc);
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
   Brain.Screen.setFillColor(yellow);
 }else if(temp >= 60){
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
 float heading = 0.0;
 float accuracy = 2.0;
 float error = target-heading;
 float kp = 2.0;
 float speed = kp * error;
 gyroT.setRotation(0.0, deg);


 while(fabs(error) >= accuracy)
 {
   speed=kp*error;
   driveTank(speed, -speed, 10);
   heading = gyroT.rotation(deg);
   error = target - heading;
   Brain.Screen.printAt(10,30, "error= %0.2f", error);
 }
 drivebrake();
}


void inchdrive(float target)
{
 float x = 0.0;
 float accuracy = 1.0;
 float error = target - x;
 float kp = 7.06;
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
     IR.spin(reverse, Ispeed, pct);
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



// =============================================================================
// AUTON SELECTOR CODE - START
// =============================================================================
// =============================================================================
// SCREEN LAYOUT CONSTANTS - Define positions and sizes for buttons
// =============================================================================
const int BUTTON_X = 10;           // X position for all buttons
const int BUTTON_WIDTH = 200;      // Width of all buttons
const int BUTTON_HEIGHT = 80;      // Height of all buttons

const int BUTTON_TOP_Y = 50;       // Y position for top button
const int BUTTON_BOTTOM_Y = 150;   // Y position for bottom button

const int SCREEN_WIDTH = 480;      // V5 Brain screen width
const int SCREEN_HEIGHT = 272;     // V5 Brain screen height

const int TEXT_X = 10;             // X position for text
const int TEXT_SELECTED_Y = 60;    // Y position for "Selected:" text
const int TEXT_COLOR_Y = 100;      // Y position for color text
const int TEXT_SIDE_Y = 140;       // Y position for side text
const int TEXT_GOAL_Y = 180;       // Y position for goal text

const int DEBOUNCE_TIME = 200;     // Milliseconds to wait after button press
const int LOOP_DELAY = 20;         // Milliseconds between loop iterations
const int DISPLAY_TIME = 2000;     // Milliseconds to display final selection

// =============================================================================
// AUTONOMOUS SELECTION SYSTEM
// =============================================================================

// This variable stores which autonomous routine to run (1-8)
int autonToRun = 0;

// Constants that represent each possible autonomous routine
const int RED_RIGHT_CENTER = 1;
const int RED_RIGHT_LONG = 2;
const int RED_LEFT_CENTER = 3;
const int RED_LEFT_LONG = 4;
const int BLUE_RIGHT_CENTER = 5;
const int BLUE_RIGHT_LONG = 6;
const int BLUE_LEFT_CENTER = 7;
const int BLUE_LEFT_LONG = 8;

// Enums define named options instead of using 0, 1, 2, etc.

// =============================================================================
// HELPER FUNCTION 1: createButton
// =============================================================================
void createButton(const vex::color &fillColor, int x, int y, int width, int height, const char *label) {
  Brain.Screen.setFillColor(fillColor);
  Brain.Screen.drawRectangle(x, y, width, height);
  Brain.Screen.setPenColor(white);
  Brain.Screen.printAt(x + 10, y + height / 2, label);
}

// =============================================================================
// HELPER FUNCTION 2: buttonPressed
// =============================================================================
bool buttonPressed(int x, int y, int width, int height) {
  if(Brain.Screen.pressing()) {
    int touchX = Brain.Screen.xPosition();
    int touchY = Brain.Screen.yPosition();
    return (touchX >= x && touchX <= x + width && touchY >= y && touchY <= y + height);
  }
  return false;
}

// =============================================================================
// SELECTION FUNCTION 1: selectColor
// =============================================================================
Color selectColor() {
  Brain.Screen.clearScreen();
  Brain.Screen.setFont(fontType::mono20);
  
  createButton(red, BUTTON_X, BUTTON_TOP_Y, BUTTON_WIDTH, BUTTON_HEIGHT, "RED");
  createButton(blue, BUTTON_X, BUTTON_BOTTOM_Y, BUTTON_WIDTH, BUTTON_HEIGHT, "BLUE");
  
  while(true) {
    if(buttonPressed(BUTTON_X, BUTTON_TOP_Y, BUTTON_WIDTH, BUTTON_HEIGHT)) {
      wait(DEBOUNCE_TIME, msec);
      return RED;
    }
    if(buttonPressed(BUTTON_X, BUTTON_BOTTOM_Y, BUTTON_WIDTH, BUTTON_HEIGHT)) {
      wait(DEBOUNCE_TIME, msec);
      return BLUE;
    }
    wait(LOOP_DELAY, msec);
  }
}

// =============================================================================
// SELECTION FUNCTION 2: selectSide
// =============================================================================
Side selectSide() {
  Brain.Screen.clearScreen();
  Brain.Screen.setFont(fontType::mono20);
  
  createButton(green, BUTTON_X, BUTTON_TOP_Y, BUTTON_WIDTH, BUTTON_HEIGHT, "RIGHT");
  createButton(orange, BUTTON_X, BUTTON_BOTTOM_Y, BUTTON_WIDTH, BUTTON_HEIGHT, "LEFT");
  
  while(true) {
    if(buttonPressed(BUTTON_X, BUTTON_TOP_Y, BUTTON_WIDTH, BUTTON_HEIGHT)) {
      wait(DEBOUNCE_TIME, msec);
      return RIGHT;
    }
    if(buttonPressed(BUTTON_X, BUTTON_BOTTOM_Y, BUTTON_WIDTH, BUTTON_HEIGHT)) {
      wait(DEBOUNCE_TIME, msec);
      return LEFT;
    }
    wait(LOOP_DELAY, msec);
  }
}

// =============================================================================
// SELECTION FUNCTION 3: selectGoal
// =============================================================================
Goal selectGoal() {
  Brain.Screen.clearScreen();
  Brain.Screen.setFont(fontType::mono20);
  
  createButton(purple, BUTTON_X, BUTTON_TOP_Y, BUTTON_WIDTH, BUTTON_HEIGHT, "CENTER");
  createButton(yellow, BUTTON_X, BUTTON_BOTTOM_Y, BUTTON_WIDTH, BUTTON_HEIGHT, "LONG");
  
  while(true) {
    if(buttonPressed(BUTTON_X, BUTTON_TOP_Y, BUTTON_WIDTH, BUTTON_HEIGHT)) {
      wait(DEBOUNCE_TIME, msec);
      return CENTER;
    }
    if(buttonPressed(BUTTON_X, BUTTON_BOTTOM_Y, BUTTON_WIDTH, BUTTON_HEIGHT)) {
      wait(DEBOUNCE_TIME, msec);
      return LONG;
    }
    wait(LOOP_DELAY, msec);
  }
}

// =============================================================================
// MAIN SELECTOR FUNCTION: autonSelector
// =============================================================================
void autonSelector() {
  // Step 1-3: Get user's three choices
  Color color = selectColor();
  Side side = selectSide();
  Goal goal = selectGoal();
  
  // Store selected color globally for color sensor filtering during match
  selectedColor = color;
  
  // Map the combination of choices to a specific autonomous routine
  if(color == RED && side == RIGHT && goal == CENTER) autonToRun = RED_RIGHT_CENTER;
  if(color == RED && side == RIGHT && goal == LONG) autonToRun = RED_RIGHT_LONG;
  if(color == RED && side == LEFT && goal == CENTER) autonToRun = RED_LEFT_CENTER;
  if(color == RED && side == LEFT && goal == LONG) autonToRun = RED_LEFT_LONG;
  if(color == BLUE && side == RIGHT && goal == CENTER) autonToRun = BLUE_RIGHT_CENTER;
  if(color == BLUE && side == RIGHT && goal == LONG) autonToRun = BLUE_RIGHT_LONG;
  if(color == BLUE && side == LEFT && goal == CENTER) autonToRun = BLUE_LEFT_CENTER;
  if(color == BLUE && side == LEFT && goal == LONG) autonToRun = BLUE_LEFT_LONG;
  
  // Display confirmation screen with alliance-colored background
  Brain.Screen.clearScreen();
  
  if(color == RED) {
    Brain.Screen.setFillColor(red);
  } else {
    Brain.Screen.setFillColor(blue);
  }
  Brain.Screen.drawRectangle(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);
  
  Brain.Screen.setPenColor(white);
  Brain.Screen.setFont(fontType::mono20);
  
  const char* colorStr = (color == RED) ? "RED" : "BLUE";
  const char* sideStr = (side == RIGHT) ? "RIGHT" : "LEFT";
  const char* goalStr = (goal == CENTER) ? "CENTER" : "LONG";
  
  Brain.Screen.printAt(TEXT_X, TEXT_SELECTED_Y, "Selected:");
  Brain.Screen.printAt(TEXT_X, TEXT_COLOR_Y, colorStr);
  Brain.Screen.printAt(TEXT_X, TEXT_SIDE_Y, sideStr);
  Brain.Screen.printAt(TEXT_X, TEXT_GOAL_Y, goalStr);
  
  wait(DISPLAY_TIME, msec);
}

// =============================================================================
// AUTON SELECTOR CODE - END
// =============================================================================

/*---------------------------------------------------------------------------*/
/*                          Autonomous Task                                  */
/*---------------------------------------------------------------------------*/

// ----- SIMPLE CONSTANTS STUDENTS CAN TUNE -----
const double WHEEL_DIAM = 3.25;         // wheel size in inches
const double PI_VAL     = 3.14159;
const double GYRO_KP    = 2.0;          // heading correction gain
const double DEGREES_PER_INCH = 27.0;  // wheel circumference

// -----------------------------------------------
// --------------- AUTON PARAMS ------------------
// -----------------------------------------------
const int FORWARD_SPEED = 30;
const int TURN_SPEED = 30;
const int SCORING_SPEED = 45;
const int WAIT_BETWEEN_ACTIONS = 100;  // in milliseconds

double degreesPerInch() {
  return DEGREES_PER_INCH;
}

// Drive straight using all 4 motors + gyro heading correction
void driveForwardInches(double inches, int speedPct) {
  LF.resetPosition();
  RF.resetPosition();
  LB.resetPosition();
  RB.resetPosition();

  double targetDeg = inches * degreesPerInch();
  double startAngle = gyroT.heading();

  while (true) {
    double avgPos = (LF.position(degrees) +
                     RF.position(degrees) +
                     LB.position(degrees) +
                     RB.position(degrees)) / 4.0;

    if (fabs(avgPos) >= fabs(targetDeg)) {
      break;
    }

    double currentAngle = gyroT.heading();
    double error = startAngle - currentAngle;

    if (error > 180)  error -= 360;
    if (error < -180) error += 360;

    double correction = error * GYRO_KP;

    double leftPower  = speedPct + correction;
    double rightPower = speedPct - correction;

    if (leftPower  > 100) leftPower  = 100;
    if (leftPower  < -100) leftPower = -100;
    if (rightPower > 100) rightPower = 100;
    if (rightPower < -100) rightPower = -100;

    LF.spin(forward, leftPower,  pct);
    LB.spin(forward, leftPower,  pct);
    RF.spin(forward, rightPower, pct);
    RB.spin(forward, rightPower, pct);

    wait(20, msec);
  }

  drivebrake();
  wait(WAIT_BETWEEN_ACTIONS, msec);
}

void backToWallSlow() {
  LF.spin(reverse, 20, pct);
  LB.spin(reverse, 20, pct);
  RF.spin(reverse, 20, pct);
  RB.spin(reverse, 20, pct);
  wait(700, msec);
  drivebrake();
}

void turnToAngle(double targetAngle, int baseSpeedPct) {
  double accuracy = 2.0;
  double kP       = 1.2;

  while (true) {
    double current = gyroT.rotation(deg);
    double error   = targetAngle - current;

    if (error > 180)  error -= 360;
    if (error < -180) error += 360;

    if (fabs(error) < accuracy) {
      break;
    }

    double speed = kP * error;

    if (speed >  baseSpeedPct)  speed =  baseSpeedPct;
    if (speed < -baseSpeedPct)  speed = -baseSpeedPct;

    driveTank(speed, -speed, 10);
  }

  drivebrake();
  wait(WAIT_BETWEEN_ACTIONS, msec);
}

void strafeRightInches(double inches, int speedPct) {
  LF.resetPosition();
  RF.resetPosition();
  LB.resetPosition();
  RB.resetPosition();

  double targetDeg = inches * degreesPerInch();
  double startAngle = gyroT.heading();

  while (true) {
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

    auto clamp = [](double v) {
      if (v > 100) return 100.0;
      if (v < -100) return -100.0;
      return v;
    };

    flPower = clamp(flPower);
    frPower = clamp(frPower);
    lbPower = clamp(lbPower);
    rbPower = clamp(rbPower);

    LF.spin(forward, flPower, pct);
    RF.spin(forward, frPower, pct);
    LB.spin(forward, lbPower, pct);
    RB.spin(forward, rbPower, pct);

    wait(20, msec);
  }

  drivebrake();
  wait(WAIT_BETWEEN_ACTIONS, msec);
}

// =============================================================================
// AUTONOMOUS ROUTINES - ALL 8 VARIANTS
// =============================================================================

void BlueLeftCenter() {
  driveForwardInches(12, FORWARD_SPEED);
  turnToAngle(-45, TURN_SPEED);
  Intake(80, 0);
  driveForwardInches(17.5, FORWARD_SPEED * 0.5);
  driveForwardInches(1.75, -FORWARD_SPEED);
  Dejam(100);
  wait(0.5, sec);
  turnToAngle(45, TURN_SPEED);     
  driveForwardInches(14, FORWARD_SPEED);
  Middlescore(SCORING_SPEED*1.5, 0);
  wait(1.5, sec);
  Dejam(100);
  wait(0.5, sec);
  Middlescore(SCORING_SPEED*1.5, 0);
  wait(1.5, sec);
  Dejam(100);
  wait(0.5, sec);
  Middlescore(SCORING_SPEED*1.5, 0);
}

void BlueLeftLong() {
  driveForwardInches(12, FORWARD_SPEED);
  turnToAngle(-45, TURN_SPEED);
  Intake(80, 0);
  driveForwardInches(17.5, FORWARD_SPEED * 0.5);
  Dejam(100);
  wait(0.5, sec);
  strafeRightInches(31.85, -FORWARD_SPEED);
  turnToAngle(0, TURN_SPEED);
  driveForwardInches(10.25, FORWARD_SPEED);
  Topscore(SCORING_SPEED*3, 0);
  wait(1.5, sec);
  Dejam(100);
  wait(0.5, sec);
  Topscore(SCORING_SPEED*3, 0);
}

void BlueRightCenter() {
  driveForwardInches(12, FORWARD_SPEED*3);
  turnToAngle(45, TURN_SPEED);
  Intake(80, 0);
  driveForwardInches(20, FORWARD_SPEED * 3);
  driveForwardInches(5, -FORWARD_SPEED*3);
  Dejam(100);
  wait(0.5, sec);
  turnToAngle(-45, TURN_SPEED);     
  driveForwardInches(17, FORWARD_SPEED*3);
  driveForwardInches(2, -FORWARD_SPEED*3);
  Bottomscore(SCORING_SPEED*2, 0);
  wait(1.5, sec);
  Dejam(100);
  wait(0.5, sec);
  Bottomscore(SCORING_SPEED*2, 0);
  wait(1.5, sec);
  Dejam(100);
  wait(0.5, sec);
  Bottomscore(SCORING_SPEED*2, 0);
  driveForwardInches(36, -FORWARD_SPEED*2);
}

void BlueRightLong() {
  driveForwardInches(12, FORWARD_SPEED*2);
  turnToAngle(45, TURN_SPEED*2);
  Intake(80, 0);
  driveForwardInches(20, FORWARD_SPEED * 1.65);
  wait(0.75, sec);
  driveForwardInches(2.5, -FORWARD_SPEED*3);
  Dejam(100);
  wait(0.75, sec);
  strafeRightInches(29, FORWARD_SPEED*3);
  turnToAngle(0, TURN_SPEED*2);
  driveForwardInches(12, FORWARD_SPEED*3);
  Topscore(SCORING_SPEED*2, 0);
  wait(1.5, sec);
  Dejam(100);
  wait(0.5, sec);
  Topscore(SCORING_SPEED*2, 0);
}

void RedLeftCenter() {
  driveForwardInches(12, FORWARD_SPEED);
  turnToAngle(-45, TURN_SPEED);
  Intake(80, 0);
  driveForwardInches(17.5, FORWARD_SPEED * 0.5);
  driveForwardInches(1.75, -FORWARD_SPEED);
  Dejam(100);
  wait(0.5, sec);
  turnToAngle(45, TURN_SPEED);     
  driveForwardInches(14, FORWARD_SPEED);
  Middlescore(SCORING_SPEED*1.5, 0);
  wait(1.5, sec);
  Dejam(100);
  wait(0.5, sec);
  Middlescore(SCORING_SPEED*1.5, 0);
  wait(1.5, sec);
  Dejam(100);
  wait(0.5, sec);
  Middlescore(SCORING_SPEED*1.5, 0);
}

void RedLeftLong() {
  driveForwardInches(12, FORWARD_SPEED);
  turnToAngle(-45, TURN_SPEED);
  Intake(80, 0);
  driveForwardInches(17.5, FORWARD_SPEED * 0.5);
  Dejam(100);
  wait(0.5, sec);
  strafeRightInches(32.65, -FORWARD_SPEED);
  turnToAngle(0, TURN_SPEED);
  driveForwardInches(9.95, FORWARD_SPEED);
  Topscore(SCORING_SPEED*3, 0);
  wait(1.5, sec);
  Dejam(100);
  wait(0.5, sec);
  Topscore(SCORING_SPEED*3, 0);
}

void RedRightCenter() {
  driveForwardInches(12, FORWARD_SPEED);
  turnToAngle(45, TURN_SPEED);
  Intake(80, 0);
  driveForwardInches(20, FORWARD_SPEED * 0.5);
  driveForwardInches(5, -FORWARD_SPEED);
  Dejam(100);
  wait(0.5, sec);
  turnToAngle(-45, TURN_SPEED);     
  driveForwardInches(17, FORWARD_SPEED);
  driveForwardInches(2, -FORWARD_SPEED);
  Bottomscore(SCORING_SPEED*2, 0);
  wait(1.5, sec);
  Dejam(100);
  wait(0.5, sec);
  Bottomscore(SCORING_SPEED*2, 0);
  wait(1.5, sec);
  Dejam(100);
  wait(0.5, sec);
  Bottomscore(SCORING_SPEED*2, 0);
}

void RedRightLong() {
  driveForwardInches(12, FORWARD_SPEED);
  turnToAngle(45, TURN_SPEED);
  Intake(80, 0);
  driveForwardInches(17.5, FORWARD_SPEED * 0.5);
  Dejam(100);
  wait(0.5, sec);
  strafeRightInches(29, FORWARD_SPEED);
  turnToAngle(0, TURN_SPEED);
  driveForwardInches(10, FORWARD_SPEED);
  Topscore(SCORING_SPEED*2, 0);
  wait(1.5, sec);
  Dejam(100);
  wait(0.5, sec);
  Topscore(SCORING_SPEED*2, 0);
}

// =============================================================================
// SWITCH STATEMENT - EXECUTES SELECTED AUTONOMOUS
// =============================================================================
void switchMatchAuton(){
    switch(autonToRun) {
    case RED_RIGHT_CENTER:
      RedRightCenter();
      break;
    case RED_RIGHT_LONG:
      RedRightLong();
      break;
    case RED_LEFT_CENTER:
      RedLeftCenter();
      break;
    case RED_LEFT_LONG:
      RedLeftLong();
      break;
    case BLUE_RIGHT_CENTER:
      BlueRightCenter();
      break;
    case BLUE_RIGHT_LONG:
      BlueRightLong();
      break;
    case BLUE_LEFT_CENTER:
      BlueLeftCenter();
      break;
    case BLUE_LEFT_LONG:
      BlueLeftLong();
      break;
  }
}

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
 // All activities that occur before the competition starts
  autonSelector();
  gyroT.calibrate();
  while (gyroT.isCalibrating());
  wait (50, msec);
  Deloader.set(false);
}

void autonomous(void) {
  switchMatchAuton();
}


/*---------------------------------------------------------------------------*/
/*                          User Control Task                                */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // Brain.Screen.clearScreen();
  Brain.resetTimer();
  THESENSOR.setLight(ledState::on);
  int Ispeed = 80;
  bool vexc = false;

  while (1) {
  //  Display();
   colorsensor(vexc);

   if( Controller1.ButtonA.pressing()) {
    colorintake(Ispeed);
    vexc = true;
   }
   else if( Controller1.ButtonB.pressing()) {
      IL.spin(reverse, Ispeed, pct);
      IR.spin(forward, Ispeed, pct);
      wait(100, msec);
      IL.spin(forward, Ispeed, pct);
      IR.spin(forward, Ispeed, pct);
      vexc = false;
   }
   else if( Controller1.ButtonY.pressing()) {
     IL.spin(reverse, Ispeed, pct);
     IR.spin(reverse, Ispeed, pct);
     IR2.spin(forward, 75, pct);
     wait(100, msec);
     IL.spin(forward, Ispeed, pct);
     IR.spin(reverse, Ispeed, pct);
     IR2.spin(forward, 75, pct);
     vexc = false;
   }
   else if(Controller1.ButtonX.pressing()) {
     vexc = false;
     IL.spin(reverse, Ispeed, pct);
     IR.spin(reverse, Ispeed, pct);
     IL2.spin(forward, 70, pct);
     IR2.spin(reverse, 75, pct);
     wait(100, msec);
     IL.spin(forward, Ispeed, pct);
     IR.spin(reverse, Ispeed, pct);
     IL2.spin(forward, 70, pct);
     IR2.spin(reverse, 75, pct);

   }
  else if (Controller1.ButtonLeft.pressing()) {
     vexc = false;
     IL.stop(brake);
     IR.stop(brake);
     IR2.stop(brake);
     IL2.stop(brake);
  }
   else if(Controller1.ButtonR2.pressing()) {
     vexc = false;
     Descorer.set(true);
   }
   else if(Controller1.ButtonR1.pressing()) {
    Descorer.set(false);
   }
   else if(Controller1.ButtonL2.pressing()) {
     Deloader.set(true);
     vexc = false;
   }
   else if(Controller1.ButtonL1.pressing()){
     Deloader.set(false);
     vexc = false;
   }

   int Y = Controller1.Axis3.position();
   int X = Controller1.Axis4.position();
   int R = Controller1.Axis1.position();
  
   LF.spin(forward, Y + X + R, percent);
   LB.spin(forward, Y - X + R, percent);
   RF.spin(forward, Y - X - R, percent);
   RB.spin(forward, Y + X - R, percent);

   wait(20, msec);
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
