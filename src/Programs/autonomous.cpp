#include "main.h"
#include "roboconfig.h"

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


// Get 2 triballs to goal and touch the bar
void Autonomous::Type1() {
    /**/
    Autonomous::Type2(); 

    // Touching the bar 

    PID::faceHeading(190);
    Drivetrain::setLeftSpeed(-200); 
    Drivetrain::setRightSpeed(-200); 
    pros::delay(900); 
    Drivetrain::brake();

    
    PID::faceHeading(225);
    Drivetrain::setLeftSpeed(-200); 
    Drivetrain::setRightSpeed(-200); 
    pros::delay(1300); 
    Drivetrain::brake();


}



// Get 2 triballs 
void Autonomous::Type2() {
    /*
    flaps.set_value(true);  
    pros::delay(200); 
    Drivetrain::setLeftSpeed(-250);
    Drivetrain::setRightSpeed(-50); 
    pros::delay(750); 
    Drivetrain::brake(); 
    flaps.set_value(false);

    PID::faceHeading(325);
    Drivetrain::setLeftSpeed(-400); 
    Drivetrain::setRightSpeed(-400); 
    pros::delay(600); 
    Drivetrain::setLeftSpeed(250); 
    Drivetrain::setRightSpeed(250); 
    pros::delay(600); 
    Drivetrain::brake();

    PID::faceHeading(340);
    Drivetrain::setLeftSpeed(-400); 
    Drivetrain::setRightSpeed(-400); 
    pros::delay(600); 
    Drivetrain::setLeftSpeed(250); 
    Drivetrain::setRightSpeed(250); 
    pros::delay(600); 
    Drivetrain::brake();
    */

   vWings.set_value(true); 
   pros::delay(200);
   Drivetrain::setLeftSpeed(-100);
   Drivetrain::setRightSpeed(-300);
   pros::delay(350);
   Drivetrain::brake();

   PID::faceHeading(20); 
    Drivetrain::setLeftSpeed(-400); 
    Drivetrain::setRightSpeed(-400); 
    pros::delay(600); 
    Drivetrain::setLeftSpeed(250); 
    Drivetrain::setRightSpeed(250); 
    pros::delay(600); 
    Drivetrain::brake();


   PID::faceHeading(30); 
    Drivetrain::setLeftSpeed(-400); 
    Drivetrain::setRightSpeed(-400); 
    pros::delay(600); 
    Drivetrain::setLeftSpeed(250); 
    Drivetrain::setRightSpeed(250); 
    pros::delay(600); 
    Drivetrain::brake();

}



// Get 3 triballs into goal
void Autonomous::Type3() {
    
    Autonomous::Type2(); 


   PID::faceHeading(270); 
   Drivetrain::setLeftSpeed(-400); 
   Drivetrain::setRightSpeed(-400); 
   pros::delay(1000); 
   Drivetrain::setLeftSpeed(-200); 
   Drivetrain::setRightSpeed(-200); 
   pros::delay(1000);   
   Drivetrain::brake(); 
   pros::delay(500); 

   PID::faceHeading(60); 
   vWings.set_value(true); 

}

void Autonomous::far() {
    
    vWings.set_value(true);
    Drivetrain::setLeftSpeed(-250);
    Drivetrain::setRightSpeed(-150); 
    pros::delay(750); 
    Drivetrain::brake(); 
    vWings.set_value(false);
    Drivetrain::setLeftSpeed(-300);
    Drivetrain::setRightSpeed(-300); 
    pros::delay(500); 
    Drivetrain::brake();    
    Drivetrain::setLeftSpeed(200);
    Drivetrain::setRightSpeed(200); 
    pros::delay(300); 
    Drivetrain::brake();    
    Drivetrain::setLeftSpeed(-600);
    Drivetrain::setRightSpeed(-600); 
    pros::delay(300); 
    Drivetrain::brake();    
    
}

void Autonomous::near() {
    Drivetrain::setLeftSpeed(-150);
    Drivetrain::setRightSpeed(-250); 
    pros::delay(750); 
    Drivetrain::brake(); 
    Drivetrain::setLeftSpeed(-600);
    Drivetrain::setRightSpeed(-600); 
    pros::delay(300); 
    Drivetrain::brake();
}

void Autonomous::improvedFar() {
    Autonomous::far(); 
    IMUSensor::reset();

  Drivetrain::setLeftSpeed(300);
    Drivetrain::setRightSpeed(300); 
    pros::delay(400); 
    Drivetrain::brake(); 

    PID::faceHeading(95);

    Intake::moveAtSpeed(200); 
    pros::delay(100); 
    Intake::stop(); 

    Intake::moveAtSpeed(-200); 

    Drivetrain::setLeftSpeed(300);
    Drivetrain::setRightSpeed(300); 
    pros::delay(1400); 
    Drivetrain::brake(); 

    pros::delay(500);
    Intake::stop(); 
    PID::turn(150);

    Intake::moveAtSpeed(200);
    pros::delay(500);
    Intake::stop();


  Drivetrain::setLeftSpeed(600);
    Drivetrain::setRightSpeed(600); 
    pros::delay(1000); 
    Drivetrain::brake(); 
}

void Autonomous::hangingTest() {
    Drivetrain::setLeftSpeed(-5 * Inertial.get_roll());
    Drivetrain::setRightSpeed(-5 * Inertial.get_roll());
}