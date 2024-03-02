#include "main.h"


extern pros::Controller master;


extern pros::Motor left_front_motor;
extern pros::Motor left_middle_motor;
extern pros::Motor left_back_motor;
extern pros::Motor right_front_motor;
extern pros::Motor right_middle_motor;
extern pros::Motor right_back_motor;

extern pros::MotorGroup left_side_motors;
extern pros::MotorGroup right_side_motors;


extern pros::Motor intake_motor;


extern pros::Motor puncher;
extern pros::Motor puncherHalf;


extern pros::ADIDigitalOut vWings;
extern pros::ADIDigitalOut hWings;
extern pros::ADIDigitalOut hang;


extern pros::IMU Inertial;

extern lemlib::Drivetrain_t drive;
extern lemlib::OdomSensors_t sensors;
extern lemlib::ChassisController_t linearController;
extern lemlib::ChassisController_t angularController;
extern lemlib::Chassis chassis;