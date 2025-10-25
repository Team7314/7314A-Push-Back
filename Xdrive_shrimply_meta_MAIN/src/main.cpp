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


	if (LF.installed()){
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

}

void drive(int Lspeed,int Rspeed, int wt){
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
  float kp = 5.0;
  float speed = kp * error;
  gyroT.setRotation(0.0, deg); // resets gyro to 0 degrees

  while(fabs(error) >= accuracy) // fabs(error) = float absolute value(error)
  {
    speed=kp*error;
    drive(speed, -speed, 10);
    heading = gyroT.rotation(deg);
    error = target - heading;
    Brain.Screen.printAt(10,30, "error= %0.2f", error);
  }
  drive(0, 0, 0);
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
  float kp = 7.06;
  float speed = kp * error;
  LF.setPosition(0, rev);

  while (fabs(error) >= accuracy)
  {
    speed = kp * error;
    drive(10, 10, 10);
    x = LF.position(rev) * 3.25 * 3.14 * 0.6;
    error = target - x;
  }
  drivebrake();
}




// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
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

void autonomous(void) {
  
  
 inchdrive(10);
 wait(500, msec);
 gyroturn(-90);
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
 
  }

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                  b.                          */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  
  Brain.resetTimer();
   int Ispeed = 80;
  // User control code here, inside the loop
  //int T = 0;
  while (1) {
    Display();
    wait (1, msec);
       
    if( Controller1.ButtonA.pressing()) {
        IR.spin(reverse, Ispeed, pct);
        IR2.spin(reverse, Ispeed, pct);
    }
    else if( Controller1.ButtonX.pressing()) {
        IL.spin(forward, Ispeed, pct);
        IR.spin(forward, Ispeed, pct);
    }
    else if( Controller1.ButtonY.pressing()) {
      IL.spin(forward, Ispeed, pct);
      IR.spin(reverse, Ispeed, pct);
      IR2.spin(forward, 40, pct);
    }
      
    else if(Controller1.ButtonB.pressing()) {
      IL.spin(forward, Ispeed, pct);
      IR.spin(reverse, Ispeed, pct);
      IL2.spin(forward, Ispeed, pct);
      IR2.spin(reverse, Ispeed, pct);
    }
    else if(Controller1.ButtonR2.pressing()) {
      IL.stop(brake);
      IR.stop(brake);
      IR2.stop(brake);
      IL2.stop(brake);
    }
    

    int Y = Controller1.Axis3.position(); // Forward/Backward
    int X = Controller1.Axis4.position(); // Left and Right
    int R = Controller1.Axis1.position(); // Rotational

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