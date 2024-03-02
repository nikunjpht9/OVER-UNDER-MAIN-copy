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


void screen2()
{
    while(true)
    {
        lemlib::Pose pose = chassis.getPose();
        pros::lcd::print(0, "x: %f", pose.x);
        pros::lcd::print(1, "y: %f", pose.y);
        pros::lcd::print(2, "heading: %f", pose.theta);
        pros::delay(10);
    }
}
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    // Initialize screen
    pros::lcd::initialize();
    //Odometry::reset();
    chassis.calibrate();
    chassis.setPose(0,0,0);
    pros::Task screenTask(screen2);
    //IMUSensor::calibrate();
    //lcd::register_btn1_cb(LCD::on_center_button);
    //lcd::register_btn0_cb(LCD::on_left_button);
    //lcd::register_btn2_cb(LCD::on_right_button);    
}



/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}


/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}


/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

//LOOK DOWN FOR FARSIDE AUTON

void autonomous() {
   // pros::delay(1000000000);
   // Intake::moveAtSpeed(200);
    pros::delay(300);
   // Intake::stop();
    //Intake::moveAtSpeed(-200);
    chassis.moveTo(0, -25.3, 1000);
    chassis.turnTo(-5.58, -32.03, 1000, true);
    chassis.moveTo(-5.58, -32.03, 1000);
    chassis.turnTo(0.34, -26.13, 1000);
    chassis.moveTo(0.34, -26.13, 1000);
    chassis.turnTo(4.02, -11.8, 1000);
    chassis.moveTo(4.02, -11.8, 1000);
    chassis.turnTo(4.35, -3.87, 1000);
    chassis.moveTo(4.35, -3.87, 1000);
    vWings.set_value(true);
    chassis.turnTo(3.11, -0.33, 1000);
    chassis.moveTo(3.11, -0.33, 1000);
    /*
    chassis.turnTo(-0.92, -8.27, 1000);
    chassis.moveTo(-0.92, -8.27, 1000);
    vWings.set_value(true);
    chassis.turnTo(-3.48, -3.11, 1000);
    vWings.set_value(false);
    chassis.moveTo(-3.48, -3.11, 1000);
    */
}

//LOOK DOWN FOR FARSIDE AUTON
//FARSIDE LEMLIB

/*
void autonomous() {
    Intake::moveAtSpeed(200);
    pros::delay(300);
    Intake::stop();
    Intake::moveAtSpeed(-200);
    chassis.moveTo(0, 7.5, 1000);
    pros::delay(250);
    chassis.moveTo(0,-21.797, 1000);
    chassis.turnTo(10,-45, 1000,true);
    vWings.set_value(true);
    chassis.moveTo(9, -37.8, 1000);
    chassis.turnTo(22.3, -43.26, 1000, true);
    vWings.set_value(false);
    chassis.moveTo(24.79, -45.95, 1000);
    Drivetrain::setVolts(-127,-127);
    pros::delay(300);
    chassis.moveTo(17.87, -43.67, 1000);
    chassis.turnTo(24.95, -43.78, 1000);
    Intake::moveAtSpeed(200);
    Drivetrain::setVolts(127,127);
    pros::delay(950);
    Intake::stop();
    chassis.moveTo(17.87, -43.67, 1000);
    chassis.turnTo(33, 3.11, 1000);
    //pros::delay(10000000000);
    Intake::moveAtSpeed(-200);
    chassis.moveTo(33, 3.11, 2000);
    Drivetrain::setVolts(-127, -127);
    pros::delay(20);
    Drivetrain::setVolts(0, 0);
    // chassis.turnTo(37.68, 7.17, 1000);
    // chassis.moveTo(37.68, 7.17, 1000);
    chassis.turnTo(35.76, -3.03, 1000);
    chassis.moveTo(35.76, -3.03, 1000);
    Intake::moveAtSpeed(200);

    // chassis.turnTo(41.47, -10.84, 1000);
    // chassis.moveTo(41.47, -10.84, 1000);



    // pros::delay(100000000000);
    // chassis.turnTo(11.29, -42.72, 1000, true);
    // chassis.moveTo(11.29, -42.72, 1000);
    // // vWings.set_value(false);
    // chassis.turnTo(19.34, -47.7, 1000, true);
    // chassis.moveTo(19, -45.8, 1000);
    // // chassis.moveTo(8.69,-36.39, 1000);
    // // chassis.turnTo(23.32,-41.94, 1000,true);
    // // chassis.moveTo(23.32,-41.94, 1000);
}

*/

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 * :)
 * If no competition control is connected, this function will run immediately
 * following initialize().
 * hi
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    // Skills::progDriver();
    while (true) {
        // LCD::display();
        Drivetrain::newControl();
        Intake::run();
        Pneumatics::hangExtend();
        Pneumatics::horizontalWingsExtend();
        Pneumatics::verticalWingsExtend();
        Puncher::punch();
        Odometry::main();
        Doctor::diagnose();
        pros::delay(20);
    }
}





