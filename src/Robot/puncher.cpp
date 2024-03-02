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


bool puncherOn = false;
bool isPuncherPressed = false;

void Puncher::punch()
{
    if(master.get_digital(DIGITAL_L2))
    {
        if(!isPuncherPressed)
        {
            puncherOn = !puncherOn;
            isPuncherPressed = true;
        }
        else 
        {
            isPuncherPressed = false;
        }
    }
    else 
    {
        isPuncherPressed = false;
    }

    
    if(puncherOn) {
        puncher.move_velocity(-100);
    }
    else {
        puncher.brake();
    }
}
