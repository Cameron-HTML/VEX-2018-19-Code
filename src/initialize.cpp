#include "main.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	lcd::initialize();
	visionSensor.clear_led();
	gyro.reset();

	Task LCDUpdateTask(LCDUpdate, &mainContainer);
	Task flyWheelPIDTask(flyWheelPID, &mainContainer);
	Task driveTrainPIDTask(driveTrainPID, &mainContainer);
	Task presetTaskTask(presetTask, &mainContainer);
	// Task flipPIDTask(flipPID, &mainContainer);
	// Task twoBarPIDTask(twoBarPID, &mainContainer);
	// Task visionTracking(tracking, &mainContainer);
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
 * on the LCDPtr->
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
	mainContainer.LCD.autoSelection = true;
}
