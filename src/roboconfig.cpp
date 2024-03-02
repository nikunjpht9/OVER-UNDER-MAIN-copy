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

#include "lemlib/api.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);

// SREES

pros::Motor left_front_motor(11, pros::E_MOTOR_GEAR_BLUE, false);
pros::Motor left_middle_motor(12, pros::E_MOTOR_GEAR_BLUE, false);
pros::Motor left_back_motor(13, pros::E_MOTOR_GEAR_BLUE, true);

pros::Motor right_front_motor(1, pros::E_MOTOR_GEAR_BLUE, true);
pros::Motor right_middle_motor(2, pros::E_MOTOR_GEAR_BLUE, true);
pros::Motor right_back_motor(3, pros::E_MOTOR_GEAR_BLUE, false);


// MINE
/*
pros::Motor left_front_motor(8, pros::E_MOTOR_GEAR_BLUE, true); //false
pros::Motor left_middle_motor(9, pros::E_MOTOR_GEAR_BLUE, true); //false
pros::Motor left_back_motor(10, pros::E_MOTOR_GEAR_BLUE, false); //true


pros::Motor right_front_motor(1, pros::E_MOTOR_GEAR_BLUE, true); //false
pros::Motor right_middle_motor(2, pros::E_MOTOR_GEAR_BLUE, false); //true
pros::Motor right_back_motor(3, pros::E_MOTOR_GEAR_BLUE, false); //true


*/

pros::Motor intake_motor(4, pros::E_MOTOR_GEAR_GREEN, false);
pros::Motor puncher(6, pros::E_MOTOR_GEAR_RED, false);
pros::Motor puncherHalf(10, pros::E_MOTOR_GEAR_GREEN, true);


pros::ADIDigitalOut vWings({{17, 'B'}});
pros::ADIDigitalOut hWings(8);
pros::ADIDigitalOut hang({{17, 'A'}});

pros::IMU Inertial(14);

pros::MotorGroup left_side_motors({left_front_motor, left_middle_motor, left_back_motor});
pros::MotorGroup right_side_motors({right_front_motor, right_middle_motor, right_back_motor});

//LEMLIB
lemlib::Drivetrain_t drive {
    &left_side_motors,
    &right_side_motors,
    11.25, //track width
    3.25, //wheel diamter
    400 // wheel rpm
};

lemlib::OdomSensors_t sensors {
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    &Inertial
};

lemlib::ChassisController_t linearController {
    40, // 100, // kP
    200, // 80, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    20// slew rate
};

lemlib::ChassisController_t angularController {
    13, // kP
    90, // kD
    1, // smallErrorRange
    100, // smallErrorTimout
    3, // largeErrorRange
    500, // largeErrorTimeout
    0 // slew rate 
};

lemlib::Chassis chassis(drive, linearController, angularController, sensors);