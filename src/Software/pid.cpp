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



// PID drive method needed




int PID::turn(double turnAngle) {


  double currentAngle, targetAngle;
  double kP = 3, kI = 0.0001, kD = 0;
  double derivative = 0;
  double integral = 0;
  double prevError = -0.000001;
  double error = 0.000001;
  double outputSpeed = 0;  
  double angleBuffer = 0.5;
  currentAngle = IMUSensor::adjustAngle(IMUSensor::globalHeading);
  targetAngle = currentAngle + turnAngle;
  while(Math::abs(targetAngle - currentAngle) > angleBuffer) {
    currentAngle = IMUSensor::adjustAngle(IMUSensor::globalHeading);
    error = targetAngle - currentAngle;
    integral += error;
    derivative = error - prevError;
    prevError = error;
    outputSpeed = error * kP + integral * kI + derivative * kD;
    if(Math::sign(error) != Math::sign(turnAngle)) {
      outputSpeed = -25 * Math::sign(turnAngle);
    }
    else if (Math::abs(error) < 1) {
      outputSpeed = 25 * Math::sign(turnAngle);
    }
    else if (Math::abs(error) < 6) {
      outputSpeed = (error + 40) * Math::sign(turnAngle);
    }
    IMUSensor::updateHeading();
    Drivetrain::setLeftSpeed(outputSpeed);
    Drivetrain::setRightSpeed(-outputSpeed);
    pros::delay(5);
  }


  Drivetrain::brake();
  return 1;
}






int PID::faceHeading(double heading_) {
  double error = heading_ - Math::convertAngle(IMUSensor::globalHeading);
  if(error >= 180) {
    error -= 360;
  }
  else if (error <= -180) {
    error += 360;
  }
  PID::turn(error);
  return 1;
}
