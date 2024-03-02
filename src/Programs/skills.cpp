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
    IMUSensor::globalHeading = 45;
    
    Drivetrain::driveAtFor(-200, -400, 400);
    Drivetrain::driveAtFor(-400, -400, 800);
    pros::delay(100);
    Drivetrain::driveAtFor(100, 400, 200);
    Drivetrain::driveAtFor(400, 100, 100);
    PID::faceHeading(165);
    hWings.set_value(true);
    puncher.move_velocity(-100);
    pros::delay(34000);
    puncher.brake();
    hWings.set_value(false);
    PID::faceHeading(45);
    Drivetrain::driveAtFor(400, 400, 700);
    PID::faceHeading(5);
    Drivetrain::driveAtFor(400, 400, 1300);
    Drivetrain::driveAtFor(200, 400, 700); 
    hWings.set_value(true);
    Drivetrain::driveAtFor(200, 400, 300);
    Drivetrain::driveAtFor(400, 400, 800);
    Drivetrain::driveAtFor(-400, -400, 300);
    Drivetrain::driveAtFor(400, 400, 500);
    Drivetrain::driveAtFor(-200, -200, 300);
    hWings.set_value(false);
    Drivetrain::driveAtFor(-400, 0, 200);
    PID::faceHeading(190);
    Drivetrain::driveAtFor(200, 200, 100);
    Drivetrain::driveAtFor(200, 100, 1000);
    hWings.set_value(true);
    Drivetrain::driveAtFor(200, 100, 3000);
    Drivetrain::driveAtFor(400, 400, 500);
    hWings.set_value(false);
    Drivetrain::driveAtFor(-200, -200, 2000);
    PID::faceHeading(0);
    hWings.set_value(true);
    Drivetrain::driveAtFor(400, 400, 1000);
    hWings.set_value(false);
    Drivetrain::driveAtFor(-200, -200, 1000);

    PID::faceHeading(90); 
    vWings.set_value(true);
    Drivetrain::driveAtFor(-300, -300, 800);
    Drivetrain::driveAtFor(-150, -300, 2400);
    vWings.set_value(false);
    Drivetrain::driveAtFor(-150, -300, 1000);
    Drivetrain::driveAtFor(300, 300, 200);
    Drivetrain::driveAtFor(-300, -300, 400);

    PID::faceHeading(270);
    Drivetrain::driveAtFor(300, 300, 200);
    Drivetrain::driveAtFor(-300, -300, 400);


}


void Skills::progDriver()
{
    while(true){
    if(master.get_digital(DIGITAL_R2))
    {
        IMUSensor::globalHeading = 45;
        Drivetrain::driveAtFor(-200, -400, 400);
        Drivetrain::driveAtFor(-400, -400, 800);
        pros::delay(100);
        Drivetrain::driveAtFor(100, 400, 200);
        Drivetrain::driveAtFor(400, 100, 100);
        PID::faceHeading(165);
        hWings.set_value(true);
        puncher.move_velocity(-100);
        pros::delay(34000);
        puncher.brake();
        hWings.set_value(false);
        PID::faceHeading(45);
        Drivetrain::driveAtFor(400, 400, 700);
        PID::faceHeading(5);
        Drivetrain::driveAtFor(400, 400, 1300);
        Drivetrain::driveAtFor(200, 400, 700); 
        hWings.set_value(true);
        Drivetrain::driveAtFor(200, 400, 300);
    }
    break;
    return;
    }
}
