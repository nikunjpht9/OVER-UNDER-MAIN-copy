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



double IMUSensor::currentHeading = 0; 
double IMUSensor::globalHeading = 0; 

double presentHeading; 
double prevHeading; 
double deltaHeading; 


void IMUSensor::calibrate() {
    Inertial.tare_heading(); 
    Inertial.reset();
    while (Inertial.is_calibrating()) {
        IMUSensor::globalHeading = infinity(); 
    }
    IMUSensor::globalHeading = 0; 
}


double IMUSensor::cartesian() {
  IMUSensor::currentHeading = Inertial.get_heading();
  return Math::headingToCartesian(IMUSensor::currentHeading); 
}


double IMUSensor::adjustedHeading() {
  // Get current heading from inertial sensor 
  IMUSensor::currentHeading = Inertial.get_heading(); 
  if(IMUSensor::currentHeading < 180) {
    return IMUSensor::currentHeading; 
  }
  else {
    return IMUSensor::currentHeading - 360; 
  }
}

void IMUSensor::updateHeading() {
  presentHeading = Inertial.get_heading();
  deltaHeading = Math::convertAngle(presentHeading - prevHeading);
  IMUSensor::globalHeading = IMUSensor::globalHeading + deltaHeading; 
  IMUSensor::globalHeading = Math::convertAngle(IMUSensor::globalHeading);
  prevHeading = presentHeading;
}

void IMUSensor::reset() {
  
IMUSensor::currentHeading = 0; 
IMUSensor::globalHeading = 0; 
presentHeading = 0; 
prevHeading = 0; 
deltaHeading = 0; 
Inertial.set_heading(0);
}