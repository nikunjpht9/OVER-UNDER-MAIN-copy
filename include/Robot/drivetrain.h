class Drivetrain {
 public:

 static void control();
 static void newControl();

 static void setLeftSpeed(double);
 static void setRightSpeed(double);

 static void setLeftVolts(double);
 static void setRightVolts(double);

 static void setLeftVoltageSpeed(double);
 static void setRightVoltageSpeed(double);


 static void brake();
 static void setBrake(pros::motor_brake_mode_e);
 
 static int LeftVelocity;
 static int RightVelocity;
 
};
