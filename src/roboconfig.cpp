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


pros::Controller master(pros::E_CONTROLLER_MASTER);

//SREES

pros::Motor left_front_motor(8, pros::E_MOTOR_GEAR_BLUE, false); 
pros::Motor left_middle_motor(9, pros::E_MOTOR_GEAR_BLUE, false); 
pros::Motor left_back_motor(10, pros::E_MOTOR_GEAR_BLUE, true); 

pros::Motor right_front_motor(1, pros::E_MOTOR_GEAR_BLUE, true); 
pros::Motor right_middle_motor(2, pros::E_MOTOR_GEAR_BLUE, true);
pros::Motor right_back_motor(3, pros::E_MOTOR_GEAR_BLUE, false); 


//MINE
/*
pros::Motor left_front_motor(8, pros::E_MOTOR_GEAR_BLUE, true); //false
pros::Motor left_middle_motor(9, pros::E_MOTOR_GEAR_BLUE, true); //false
pros::Motor left_back_motor(10, pros::E_MOTOR_GEAR_BLUE, false); //true

pros::Motor right_front_motor(1, pros::E_MOTOR_GEAR_BLUE, true); //false
pros::Motor right_middle_motor(2, pros::E_MOTOR_GEAR_BLUE, false); //true
pros::Motor right_back_motor(3, pros::E_MOTOR_GEAR_BLUE, false); //true

*/
pros::Distance distance(12);


pros::Motor intake_motor(7, pros::E_MOTOR_GEAR_GREEN, false); 
pros::Motor puncher(5, pros::E_MOTOR_GEAR_RED, false); 

pros::ADIDigitalOut vWings(1);
pros::ADIDigitalOut blocker(2);
pros::ADIDigitalOut hWings(3);

pros::IMU Inertial(11); 
