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



int LCD::autonomousSelection = 1; 
int LCD::color = 10; 

void LCD::display() {

	if(LCD::color <= 20) {
		lcd::set_background_color(LV_COLOR_MAKE(255, 127, 127)); 
	}
	else if(LCD::color <= 30) {
		lcd::set_background_color(LV_COLOR_RED);
	}
	else if(LCD::color <= 40) {
		lcd::set_background_color(LV_COLOR_ORANGE); 
	}
	else if(LCD::color <= 50) {
		lcd::set_background_color(LV_COLOR_YELLOW); 
	}
	else if(LCD::color <= 60) {
		lcd::set_background_color(LV_COLOR_GREEN); 
	}	
	else if(LCD::color <= 70) {
		lcd::set_background_color(LV_COLOR_BLUE); 
	}
	else if(LCD::color <= 80) {
		lcd::set_background_color(LV_COLOR_PURPLE); 
	}

	lcd::print(0, "AUTON: %d   LEFT: %d   RIGHT: %d", LCD::autonomousSelection, Drivetrain::LeftVelocity, Drivetrain::RightVelocity);
	lcd::print(1, "X: %d   Y: %d   HEADING: %.2f", 0, 0, IMUSensor::globalHeading); 
    lcd::print(2, "INTAKE: %.1f" , intake_motor.get_actual_velocity()); 
    lcd::print(3, "BLOCKER: %d" , Pneumatics::isBlockerOn); 
    lcd::print(4, "VERTICAL: %d" , Pneumatics::isVerticalOn); 
    lcd::print(5, "HORIZONTAL: %d" , Pneumatics::isHorizontalOn); 
	lcd::print(6, "PITCH: %.1f", Inertial.get_pitch());
	lcd::print(7, "LEFT AUTON   SKILLS   RIGHT AUTON");
	LCD::color += 1; 
	if(LCD::color == 81) {
		LCD::color = 1; 
	}
	
}

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will set autonomous type to skills mode
 */
void LCD::on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		LCD::autonomousSelection = 1; 
	} else {}
}


/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will set autonomous type to near side autonomous mode
 */

void LCD::on_left_button() {
	static bool pressed = false;
	pressed = !pressed; 
	if(pressed) {
		LCD::autonomousSelection = 0; 
	} else {} 
}



/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will set autonomous type to far side autonomous mode
 */

void LCD::on_right_button() {
	static bool pressed = false;
	pressed = !pressed; 
	if(pressed) {
		LCD::autonomousSelection = 2; 
	} else {} 
}
