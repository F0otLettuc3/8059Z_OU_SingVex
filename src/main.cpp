#include "main.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	Motor frontLeft(frontLeftPort,E_MOTOR_GEARSET_06,true,E_MOTOR_ENCODER_DEGREES);
	Motor midLeft(midLeftPort,E_MOTOR_GEARSET_06,false,E_MOTOR_ENCODER_DEGREES);
	Motor backLeft(backLeftPort,E_MOTOR_GEARSET_06,true,E_MOTOR_ENCODER_DEGREES);
	Motor frontRight(frontRightPort,E_MOTOR_GEARSET_06,false,E_MOTOR_ENCODER_DEGREES);
	Motor midRight(midRightPort,E_MOTOR_GEARSET_06,true,E_MOTOR_ENCODER_DEGREES);
	Motor backRight(backRightPort,E_MOTOR_GEARSET_06,false,E_MOTOR_ENCODER_DEGREES);

	Motor catapult(catapultPort,E_MOTOR_GEARSET_36,false,E_MOTOR_ENCODER_DEGREES);
	Rotation catapultSensor(catapultSensorPort);
	Imu imu(imuPort);

	Task catapultTask(catapultControl, (void*)"PROS",TASK_PRIORITY_DEFAULT,TASK_STACK_DEPTH_DEFAULT);
	Task odometryTask(odometry,(void*)"PROS",TASK_PRIORITY_DEFAULT,TASK_STACK_DEPTH_DEFAULT);
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
void autonomous() {
	auton = true;
	Task baseControlTask(baseControl,(void*)"PROS",TASK_PRIORITY_DEFAULT,TASK_STACK_DEPTH_DEFAULT);
	baseControlTask.remove();

}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	Controller master(E_CONTROLLER_MASTER);
	Motor frontLeft(frontLeftPort);
	Motor midLeft(midLeftPort);
	Motor backLeft(backLeftPort);
	Motor frontRight(frontRightPort);
	Motor midRight(midRightPort);
	Motor backRight(backRightPort);
	Motor catapult(catapultPort);

	Rotation catapultSensor(catapultSensorPort);

	double setTime = millis();
	auton = false;
	catapultManual = true;

	while (true) {
		//Drive Control
		double left = master.get_analog(ANALOG_LEFT_Y);
		double right = master.get_analog(ANALOG_RIGHT_Y);

		frontLeft.move(left);
		midLeft.move(left);
		backLeft.move(left);
		frontRight.move(right);
		midRight.move(right);
		backRight.move(right);
		//Testing for Straight Movement
		/*
		double centre = master.get_analog(ANALOG_LEFT_Y);
		frontLeft.move(centre);
		midLeft.move(centre);
		backLeft.move(centre);
		frontRight.move(centre);
		midRight.move(centre);
		backRight.move(centre);
		*/

		//Catapult Control
		if(master.get_digital_new_press(DIGITAL_R1)){
			catapultTarg = 0;
			setTime = millis();
		}
		if((millis()-setTime)>500){
			catapultTarg = downTarg;
			setTime = millis();
		}
		if(master.get_digital(DIGITAL_B)){
			catapultManual = true;
			catapult.move(127);
		}
		else{catapultManual = false;}
		double pos = catapultSensor.get_angle();
		//printf("catapult pos%.2f\n",pos);

		delay(20);
	}
}
