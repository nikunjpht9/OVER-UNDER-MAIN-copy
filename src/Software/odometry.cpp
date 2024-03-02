#include "main.h"
#include "roboconfig.h"
#include<math.h>


#include "Programs/skills.h"
#include "Programs/autonomous.h"


#include "Software/lcd.h"
#include "Software/doctor.h"
#include "Software/imu.h"
#include "Software/odometry.h"
#include "Software/pid.h"
#include "Software/mathfunctions.h"


#include "Robot/drivetrain.h"
#include "Robot/intake.h"
#include "Robot/puncher.h"
#include "Robot/pneumatics.h"


#define Pi 3.14159265358979323846
#define fieldscale 1.66548042705
#define SL 8 //distance from tracking center to middle of left wheel
#define SR 8 //distance from tracking center to middle of right wheel
#define SS 8 //distance from tracking center to middle of the tracking wheel
#define WheelDiam 3.125 //diameter of all the wheels being used for tracking
#define tpr 360  //Degrees per single encoder rotation
double DeltaL,DeltaR,DeltaB,currentL,currentR,PreviousL,PreviousR,DeltaTheta,X,Y,Theta,DeltaXSide,DeltaYSide,SideChord,OdomHeading;
double Odometry::X, Odometry::Y;

void Odometry::main() {
 
  currentR = (-right_front_motor.get_position() + right_back_motor.get_position()) / 2;
  currentL = (left_front_motor.get_position() - left_back_motor.get_position() ) / 2;


  DeltaL = ((currentL - PreviousL) * WheelDiam * Pi) / tpr;
  DeltaR = ((currentR - PreviousR) * WheelDiam * Pi) / tpr;


  DeltaTheta = (DeltaR - DeltaL) / (SL + SR);


  //Creates an if/else statement to prevent NaN values from appearing and causing issues with calculation
  if(DeltaTheta == 0) {  //If there is no change in angle
    Odometry::X += DeltaL * sin (Theta);
    Odometry::Y += DeltaL * cos (Theta);
    //X += DeltaB * cos (Theta + 1.57079633);
    //Y += DeltaB * sin (Theta + 1.57079633);


  //If there is a change in angle, it will calculate the changes in X,Y from chords of an arc/circle.
  } else {  //If the angle changes
      SideChord = 2 * ((DeltaL / DeltaTheta) + SL) * sin (DeltaTheta / 2);
      //BackChord = 2 * ((DeltaB / DeltaTheta) + SS) * sin (DeltaTheta / 2);
      DeltaYSide = SideChord * cos (Theta + (DeltaTheta / 2));
      DeltaXSide = SideChord * sin (Theta + (DeltaTheta / 2));
      //DeltaXBack = BackChord * sin (Theta + (DeltaTheta / 2));
      //DeltaYBack = -BackChord * cos (Theta + (DeltaTheta / 2));
      Theta += DeltaTheta;
      Odometry::X += DeltaXSide;
      Odometry::Y += DeltaYSide;
    }


    //Odom heading is converting the radian value of Theta into degrees
    OdomHeading = Theta * 57.295779513;


    //Converts values into newer values to allow for code to effectively work in next cycle
    PreviousL = currentL;
    PreviousR = currentR;
    DeltaTheta = 0;
}


void Odometry::reset() {
    right_front_motor.set_encoder_units(E_MOTOR_ENCODER_DEGREES);
    right_back_motor.set_encoder_units(E_MOTOR_ENCODER_DEGREES);
    left_front_motor.set_encoder_units(E_MOTOR_ENCODER_DEGREES);
    left_back_motor.set_encoder_units(E_MOTOR_ENCODER_DEGREES);


    Odometry::X = 0;
    Odometry::Y = 0;
}
