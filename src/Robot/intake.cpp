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


int Intake::speed_rpm = 200; 
int Intake::direction = 0; 

void Intake::run() {
    if(master.get_digital(DIGITAL_L1) && ! master.get_digital(DIGITAL_R1)) {
        Intake::direction = -1; 
    }
    else if(!master.get_digital(DIGITAL_L1) && master.get_digital(DIGITAL_R1)) {
        Intake::direction = 1; 
    }
    else {
        Intake::direction = 0; 
    }
    Intake::moveAtSpeed(Intake::speed_rpm * Intake::direction * 1.0); 
}

void Intake::moveAtSpeed(double speed) {
    intake_motor.move_velocity(speed); 
}

void Intake::stop() {
    intake_motor.brake(); 
}

