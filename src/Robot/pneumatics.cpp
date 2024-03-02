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

bool Pneumatics::isBlockerOn = false;
bool isBlockerPressed = false;
bool Pneumatics::isVerticalOn = false;
bool isVerticalPressed = false;
bool Pneumatics::isHorizontalOn = false;
bool isHorizontalPressed = false;

void Pneumatics::blockerExtend()
{
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B))
    {
        if(!isBlockerPressed)
        {
            isBlockerOn = !isBlockerOn;
            isBlockerPressed = true;
        }
    }
    else
    {
        isBlockerPressed = false;
    }
    blocker.set_value(isBlockerOn);
}

void Pneumatics::verticalWingsExtend()
{
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
    {
        if(!isVerticalPressed)
        {
            isVerticalOn = !isVerticalOn;
            isVerticalPressed = true;
        }
    }
    else
    {
        isVerticalPressed = false;
    }
    vWings.set_value(isVerticalOn);
}

void Pneumatics::horizontalWingsExtend()
{
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X))
    {
        if(!isHorizontalPressed)
        {
            isHorizontalOn = !isHorizontalOn;
            isHorizontalPressed = true;
        }
    }
    else
    {
        isHorizontalPressed = false;
    }
    hWings.set_value(isHorizontalOn);
}