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




// ONLY USE NEAR AND FAR FOR AUTONOMOUS!!


/*
// Get 2 triballs to goal and touch the bar
void Autonomous::Type1() {
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
*/




void Autonomous::far() {
    IMUSensor::globalHeading = 315;
    vWings.set_value(true);
    Drivetrain::driveAtFor(-250, -150, 750);
    vWings.set_value(false);
    Drivetrain::driveAtFor(-300, -300, 500);
    Drivetrain::driveAtFor(200, 200, 300);  
    Drivetrain::driveAtFor(-600, -600, 500);    
    Drivetrain::driveAtFor(200, 200, 300);  
}


void Autonomous::near() {
    Drivetrain::driveAtFor(-150, -250, 750);
    Drivetrain::driveAtFor(-600, -600, 400);
}

void Autonomous::farAWP() {
    IMUSensor::globalHeading = 135;
    Intake::moveAtSpeed(200);
    vWings.set_value(true);
    Drivetrain::driveAtFor(-250, -150, 750);
    Intake::moveAtSpeed(0);

    vWings.set_value(false);
    Drivetrain::driveAtFor(-300, -300, 500);
    Drivetrain::driveAtFor(200, 200, 300);  
    Drivetrain::driveAtFor(-600, -600, 500);    
    Drivetrain::driveAtFor(200, 200, 300);  
    PID::faceHeading(120);
    Drivetrain::driveAtFor(300, 300, 1200); 
    PID::faceHeading(180); 
    Drivetrain::driveAtFor(150, 150, 1200);
}

void Autonomous::farV2()
{
    IMUSensor::globalHeading = 180;
    Intake::moveAtSpeed(200);
    Drivetrain::driveAtFor(250, 250, 250);
    Intake::moveAtSpeed(-200);
}

void Autonomous::closeSide9() {
    IMUSensor::globalHeading = 225;
    Intake::moveAtSpeed(200);
    pros::delay(300);
    Intake::stop();
    Intake::moveAtSpeed(-200);
    pros::delay(300);
    Intake::stop();
    Drivetrain::driveAtFor(300,300,500);
    vWings.set_value(true);
    Drivetrain::driveAtFor(-200,-100,600);
    vWings.set_value(false);
    PID::faceHeading(225);
    Drivetrain::driveAtFor(100,100,2200);

    /*
    PID::faceHeading(250);
    Intake::moveAtSpeed(200);
    pros::delay(500);
    PID::faceHeading(250);

    Drivetrain::driveAtFor(-300,-300,800);
    Drivetrain::driveAtFor(300,300,200);
    */


    /*
    Drivetrain::drieAtFor(-300, -300, 200);
    PID::faceHeading(225);
    Drivetrain::driveAtFor(-300, -300, 1000);
    */

}




