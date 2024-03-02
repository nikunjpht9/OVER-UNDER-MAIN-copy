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


bool Pneumatics::hangDirection = false;
bool Pneumatics::isVerticalOn = false;
bool Pneumatics::isHorizontalOn = false;
bool isHorizontalPressed = false;
bool isVerticalPressed = false;
bool isHangUpPressed = false;
bool isHangDownPressed = false;


void Pneumatics::verticalWingsExtend()
{
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
    {
        if(!isVerticalPressed)
        {
            Pneumatics::isVerticalOn = !Pneumatics::isVerticalOn;
            isVerticalPressed = true;
        }
    }
    else
    {
        isVerticalPressed = false;
    }
    vWings.set_value(Pneumatics::isVerticalOn);
}


void Pneumatics::horizontalWingsExtend()
{
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X))
    {
        if(!isHorizontalPressed)
        {
            Pneumatics::isHorizontalOn = !Pneumatics::isHorizontalOn;
            isHorizontalPressed = true;
        }
    }
    else
    {
        isHorizontalPressed = false;
    }
    hWings.set_value(Pneumatics::isHorizontalOn);
}






void Pneumatics::hangExtend()
{
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_UP))
    {
        if(!isHangUpPressed)
        {
            Pneumatics::hangDirection = true;
            isHangUpPressed = true;
        }
    }
    else
    {
        isHangUpPressed = false;
    }


    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
    {
        if(!isHangDownPressed)
        {
            Pneumatics::hangDirection = false;
            isHangDownPressed = true;
        }
    }
    else
    {
        isHangDownPressed = false;
    }
    hang.set_value(Pneumatics::hangDirection);
}


