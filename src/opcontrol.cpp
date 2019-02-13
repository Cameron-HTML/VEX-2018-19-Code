#include "main.h"

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
	// Init variables
	bool reversed = false;
	bool brake = false;
	mainContainer.driveTrain.PID.PIDRunning = false;
	while(true) {

		// Joystick thresh-hold
		if(mainContainer.preset.doubleShot == false) {
			if(abs(master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y)) >= 15 || abs(master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y)) >= 15) {
				if(reversed) {
					leftFrontDriveMotor.move(master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y));
					leftBackDriveMotor.move(master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y));
					rightFrontDriveMotor.move(master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y));
					rightBackDriveMotor.move(master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y));
				} else {
					leftFrontDriveMotor.move(master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y));
					leftBackDriveMotor.move(master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y));
					rightFrontDriveMotor.move(master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y));
					rightBackDriveMotor.move(master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y));
				}
			} else {
				leftFrontDriveMotor.move(0);
				leftBackDriveMotor.move(0);
				rightFrontDriveMotor.move(0);
				rightBackDriveMotor.move(0);
			}
		}

		// Brake system
		if(master.get_digital(E_CONTROLLER_DIGITAL_DOWN)) {
			if(brake) {
				brake = false;
			} else {
				brake = true;
			}

			while(master.get_digital(E_CONTROLLER_DIGITAL_DOWN)) delay(20);
		}

		if(brake) {
			leftFrontDriveMotor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
			leftBackDriveMotor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
			rightFrontDriveMotor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
			rightBackDriveMotor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
		} else {
			leftFrontDriveMotor.set_brake_mode(E_MOTOR_BRAKE_COAST);
			leftBackDriveMotor.set_brake_mode(E_MOTOR_BRAKE_COAST);
			rightFrontDriveMotor.set_brake_mode(E_MOTOR_BRAKE_COAST);
			rightBackDriveMotor.set_brake_mode(E_MOTOR_BRAKE_COAST);
		}

		// Control 'front' of robot
		if(master.get_digital(E_CONTROLLER_DIGITAL_UP)) {
 			if(reversed) {
				reversed = false;
				leftFrontDriveMotor.set_reversed(true);
				leftBackDriveMotor.set_reversed(true);
				rightFrontDriveMotor.set_reversed(false);
				rightBackDriveMotor.set_reversed(false);
			} else {
				reversed = true;
				leftFrontDriveMotor.set_reversed(false);
				leftBackDriveMotor.set_reversed(false);
				rightFrontDriveMotor.set_reversed(true);
				rightBackDriveMotor.set_reversed(true);
			}

			while(master.get_digital(E_CONTROLLER_DIGITAL_UP)) delay(20);
		}

		// Flywheel contol
		if(master.get_digital(E_CONTROLLER_DIGITAL_R1)) {
			if(mainContainer.flyWheel.PID.PIDRunning == true) {
				mainContainer.flyWheel.PID.PIDRunning = false;
			} else {
				mainContainer.flyWheel.PID.PIDRunning = true;
			}

			while(master.get_digital(E_CONTROLLER_DIGITAL_R1)) delay(20);
		}

		// Intake/indexer control
		if(mainContainer.preset.doubleShot == false) {
			if(master.get_digital(E_CONTROLLER_DIGITAL_R2)) {
				intakeMotor.move(-127);
				indexerMotor.move(-127);
			}	else if(master.get_digital(E_CONTROLLER_DIGITAL_L2)) {
				intakeMotor.move(127);
				indexerMotor.move(127);
			} else if(master.get_digital(E_CONTROLLER_DIGITAL_L1)) {
				intakeMotor.move(-127);
			} else {
				intakeMotor.move(0);
				indexerMotor.move(0);
			}
		}

		if(master.get_digital(E_CONTROLLER_DIGITAL_X) && mainContainer.preset.doubleShot) {
			leftFrontDriveMotor.move(0);
			leftBackDriveMotor.move(0);
			rightFrontDriveMotor.move(0);
			rightBackDriveMotor.move(0);
			indexerMotor.move(0);
			mainContainer.preset.doubleShot = false;

			while(master.get_digital(E_CONTROLLER_DIGITAL_X)) delay(20);
		} else if(master.get_digital(E_CONTROLLER_DIGITAL_X)) {
			mainContainer.preset.doubleShot = true;

			while(master.get_digital(E_CONTROLLER_DIGITAL_X)) delay(20);
		}

		/* flipper PID control
		if(master.get_digital(E_CONTROLLER_DIGITAL_RIGHT)) {
			if(mainContainer.flip.PID.PIDRunning) {
				mainContainer.flip.PID.requestedValue += 180;
			} else {
				flipperMotor.move(25);
			}

			while(master.get_digital(E_CONTROLLER_DIGITAL_RIGHT)) delay(20);
		} else if(master.get_digital(E_CONTROLLER_DIGITAL_LEFT)) {
			if(mainContainer.flip.PID.PIDRunning) {
				mainContainer.flip.PID.requestedValue -= 180;
			} else {
				flipperMotor.move(-25);
			}


			while(master.get_digital(E_CONTROLLER_DIGITAL_LEFT)) delay(20);
		} else {
			if(mainContainer.flip.PID.PIDRunning == false) {
				flipperMotor.move(0);
			}
		}
		*/

		/* Two PID control
		if(master.get_digital(E_CONTROLLER_DIGITAL_R1)) {
			if(mainContainer.twoBar.PID.PIDRunning) {
				mainContainer.twoBar.PID.requestedValue += 55;
			} else {
				twoBarMotor.move(127);
			}
		} else if(master.get_digital(E_CONTROLLER_DIGITAL_R2)) {
			if(mainContainer.twoBar.PID.PIDRunning) {
				mainContainer.twoBar.PID.requestedValue -= 33;
			} else {
				twoBarMotor.move(-127);
			}
		} else {
			if(mainContainer.twoBar.PID.PIDRunning == false) {
				twoBarMotor.move(0);
			}
		}

		if(master.get_digital(E_CONTROLLER_DIGITAL_X)) {
			if(mainContainer.twoBar.PID.PIDRunning) {
				mainContainer.twoBar.PID.PIDRunning = false;
			} else {
				mainContainer.twoBar.PID.requestedValue = twoBarMotor.get_position();
				mainContainer.twoBar.PID.PIDRunning = true;
			}

			while(master.get_digital(E_CONTROLLER_DIGITAL_X)) delay(20);
		} else if(master.get_digital(E_CONTROLLER_DIGITAL_Y)) {
			if(mainContainer.flip.PID.PIDRunning) {
				mainContainer.flip.PID.PIDRunning = false;
			} else {
				mainContainer.flip.PID.requestedValue = flipperMotor.get_position();
				mainContainer.flip.PID.PIDRunning = true;
			}

			while(master.get_digital(E_CONTROLLER_DIGITAL_Y)) delay(20);
		}
		*/

		delay(20);
	}
}
