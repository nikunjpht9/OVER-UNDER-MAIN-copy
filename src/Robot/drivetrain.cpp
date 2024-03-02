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

int deadband = 5; 
double aggressiveCoeff = 1; 
double turningCoeff = 0.5;

const double deltaTime = 0.01;
const double maxMotorVoltage = 127;
const double maxVolts = 8000000000;

double currentPowerLeft = 0;
double currentPowerRight = 0;
double initialLeftPower = 0;
double initialRightPower = 0;

int Drivetrain::LeftVelocity = 0; 
int Drivetrain::RightVelocity = 0; 


// Method that sets velocities based on response from controller
void Drivetrain::control() {

   // Get controller axis values --> Analog Values
  int controllerY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
  int controllerX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
   Drivetrain::LeftVelocity = (controllerY + controllerX * turningCoeff) * 600.0/127.0 * aggressiveCoeff; 
   Drivetrain::RightVelocity = (controllerY - controllerX * turningCoeff) * 600.0/127.0 * aggressiveCoeff; 


   if (abs(Drivetrain::RightVelocity) <= deadband) {
     Drivetrain::RightVelocity = 0; 
   }
   if (abs(Drivetrain::LeftVelocity) <= deadband) {
     Drivetrain::LeftVelocity = 0; 
   }
   Drivetrain::setLeftSpeed(Drivetrain::LeftVelocity);
   Drivetrain::setRightSpeed(Drivetrain::RightVelocity);
    
  IMUSensor::updateHeading(); 
}



void Drivetrain::newControl()
{
  int controllerX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
  int controllerY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

  if(abs(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) < deadband)
  {
    controllerX = 0;
  }
  if(abs(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) < deadband)
  {
    controllerY = 0;
  }
  double desiredRightPower = controllerY - controllerX; desiredRightPower = (desiredRightPower > maxMotorVoltage) ? maxMotorVoltage : (desiredRightPower < -maxMotorVoltage) ? -maxMotorVoltage : desiredRightPower;

  double desiredLeftPower = controllerY + controllerX; desiredLeftPower = (desiredLeftPower > maxMotorVoltage) ? maxMotorVoltage : (desiredLeftPower < -maxMotorVoltage) ? -maxMotorVoltage : desiredLeftPower;

  currentPowerLeft = desiredLeftPower;
  currentPowerRight = desiredRightPower;

  double deltaLeftVolt = (currentPowerLeft - initialLeftPower ) / deltaTime; deltaLeftVolt = deltaLeftVolt > maxVolts ? maxVolts : deltaLeftVolt < -maxVolts ? -maxVolts : deltaLeftVolt;
  double deltaRightVolt = (currentPowerRight - initialRightPower) / deltaTime; deltaRightVolt = deltaRightVolt > maxVolts ? maxVolts : deltaRightVolt < -maxVolts ? -maxVolts : deltaRightVolt;

  double leftPower = initialLeftPower + (deltaLeftVolt * deltaTime);
  double rightPower = initialRightPower + (deltaRightVolt * deltaTime);

  setLeftVoltageSpeed(leftPower);
  setRightVoltageSpeed(rightPower);
}

void Drivetrain::setLeftVoltageSpeed(double Left)
{
  left_front_motor.move(Left);
  left_middle_motor.move(Left);
  left_back_motor.move(Left);
  IMUSensor::updateHeading(); 

}
void Drivetrain::setRightVoltageSpeed(double Right)
{
  right_front_motor.move(Right);
  right_middle_motor.move(Right);
  right_back_motor.move(Right);
  IMUSensor::updateHeading(); 
}


// Drive only the left side of the drivetrain at specified velocity until stopped
void Drivetrain::setLeftSpeed(double velocity) {
   left_front_motor.move_velocity(velocity);
  left_middle_motor.move_velocity(velocity); 
   left_back_motor.move_velocity(velocity);
  IMUSensor::updateHeading(); 
}

// Drive only the right side of the drivetrain at specified velocity until stopped
void Drivetrain::setRightSpeed(double velocity) {
   right_front_motor.move_velocity(velocity);
  right_middle_motor.move_velocity(velocity); 
   right_back_motor.move_velocity(velocity);
  IMUSensor::updateHeading(); 
}

void Drivetrain::setLeftVolts(double volts) {
   left_front_motor.move_voltage(volts); 
  left_middle_motor.move_voltage(volts); 
   left_back_motor.move_voltage(volts); 
 IMUSensor::updateHeading(); 
}

void Drivetrain::setRightVolts(double volts) {
   right_front_motor.move_voltage(volts); 
   right_middle_motor.move_voltage(volts);
   right_back_motor.move_voltage(volts); 
    IMUSensor::updateHeading(); 
}

void Drivetrain::brake() {
  left_front_motor.brake(); 
  left_middle_motor.brake(); 
   left_back_motor.brake(); 
   right_front_motor.brake(); 
   right_middle_motor.brake(); 
   right_back_motor.brake(); 
}

void Drivetrain::setBrake(pros::motor_brake_mode_e mode) {
   left_front_motor.set_brake_mode(mode); 
   left_middle_motor.set_brake_mode(mode); 
   left_back_motor.set_brake_mode(mode); 
   right_front_motor. set_brake_mode(mode); 
   right_middle_motor.set_brake_mode(mode); 
   right_back_motor.set_brake_mode(mode); 
}
