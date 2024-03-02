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

void Skills::main()
{
    while(true) {
        /*
        PID::faceHeading(0); 
        Drivetrain::setLeftSpeed(-600); 
        Drivetrain::setRightSpeed(-600); 
        pros::delay(1300);
        Drivetrain::brake(); 
        Drivetrain::setLeftSpeed(600); 
        Drivetrain::setRightSpeed(600); 
        pros::delay(1300);
        Drivetrain::setLeftSpeed(200); 
        Drivetrain::setRightSpeed(200); 
        pros::delay(1300);
        pros::delay(2000); 
        */

       for(int i = 0; i < 44; i++)
       {
        Intake::run();
       }
       
    }
}

