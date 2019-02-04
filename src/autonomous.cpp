#include "main.h"

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

void drive(float requested) {
  leftFrontDriveMotor.tare_position();
  leftBackDriveMotor.tare_position();
  rightFrontDriveMotor.tare_position();
  rightBackDriveMotor.tare_position();

  mainContainer.driveTrain.PID.requestedValue = requested;
  mainContainer.driveTrain.PID.PIDRunning = true;

  while(mainContainer.driveTrain.PID.PIDRunning) delay(20);

  leftFrontDriveMotor.move(0);
  leftBackDriveMotor.move(0);
  rightFrontDriveMotor.move(0);
  rightBackDriveMotor.move(0);
}

void turn(int degrees, bool right = false) {
  gyro.reset();

  int kP = 1;
  int error = 0;
  int maxPower = 127;
  int minPower = -127;
  int outputPower = 0;
  int currentSensorValue = 0;

  degrees *= 10;

  if(!right) {
    degrees *= -1;
  }

  while(gyro.get_value() < degrees) {
    // Calculate the sensor value
    currentSensorValue = gyro.get_value();

    error = currentSensorValue - degrees;

    outputPower += error / kP;

    // Limit 'outputPower'
    if(outputPower > maxPower) {
      outputPower = maxPower;
    } else if(outputPower < minPower) {
      outputPower = minPower;
    }

    delay(20);
  }

  gyro.reset();

}

void rFlywheel(bool run = true) {
  mainContainer.flyWheel.PID.PIDRunning = run;
}

/* REMEMBER TO TEST ALL OF THE BELOW */

void autonomous() {
  switch(mainContainer.LCD.auton) {
    // BLUE - LEFT
    case 0:
      drive(500);
      /*
      // Run the intake
      intakeMotor.move(127);
      // Move toward cap to intake the ball
      drive(2000);
      // Wait 1/4 of a second to ensure we have the ball
      delay(250);
      // Stop the intake motor
      intakeMotor.move(0);
      // Drive backward to align with flags
      drive(1250, false);
      // Turn on the flywheel
      rFlywheel();
      // Turn 45 degrees to face the flags
      turn(45, true);
      // Turn on the intake to push balls up to the indexer
      intakeMotor.move(127);
      // Turn on the indexer to shoot the balls
      indexerMotor.move(127);
      // Delay by half a second to ensure the balls have been shot
      delay(500);
      // Stop the intake
      intakeMotor.move(0);
      // Stop the indexer
      indexerMotor.move(0);
      // Turn off the flywheel
      rFlywheel(false);
      // Turn 45 degrees to be parallel with the platform
      turn(45);
      // Slightly move forward to center with the platform
      drive(250);
      // Turn 90 degrees to face platform
      turn(90, true);
      // Move forward to get onto platform
      drive(1000);
      */
    break;
    // BLUE - RIGHT
    case 1:
      /*
      // Turn on the intake
      intakeMotor.move(127);
      // Move forward to intake ball under cap
      drive(2000);
      // Wait 1/4 of a second to ensure we have the balls
      delay(250);
      // Turn off the intake
      intakeMotor.move(0);
      // Turn 45 degrees to face cap
      turn(45, true);
      // Reverse the intake to prepare for cap flipping
      intakeMotor.move(-127);
      // Drive forward into cap to flip
      drive(1000);
      // Drive backward to previous location
      drive(1000);
      // Turn 45 degrees to undo turn
      turn(45);
      // Drive backward to align with flags
      drive(500, false);
      // Turn on the flywheel
      rFlywheel();
      // Turn 90 degrees to face the flags
      turn(90);
      // Turn on the intake to push them into the indexer
      intakeMotor.move(127);
      // Turn on the indexer to shoot the balls
      indexerMotor.move(127);
      // Drive forward to double shot and get bottom flag
      drive(1000);
      // Turn off the flywheel
      rFlywheel(false);
      // Drive backward to align with the platform
      drive(2500, false);
      // Turn 90 degrees to face the platform
      turn(90);
      // Drive forward to get on the platform
      drive(1000);
      */
    break;
    // RED - LEFT
    case 2:
      /*
      // Turn on the intake
      intakeMotor.move(127);
      // Move forward to intake ball under cap
      drive(2000);
      // Wait 1/4 of a second to ensure we have the balls
      delay(250);
      // Turn off the intake
      intakeMotor.move(0);
      // Turn 45 degrees to face cap
      turn(45);
      // Reverse the intake to prepare for cap flipping
      intakeMotor.move(-127);
      // Drive forward into cap to flip
      drive(1000);
      // Drive backward to previous location
      drive(1000, false);
      // Turn 45 degrees to undo turn
      turn(45, true);
      // Drive backward to align with flags
      drive(500, false);
      // Turn on the flywheel
      rFlywheel();
      // Turn 90 degrees to face the flags
      turn(90, true);
      // Turn on the intake to push them into the indexer
      intakeMotor.move(127);
      // Turn on the indexer to shoot the balls
      indexerMotor.move(127);
      // Drive forward to double shot and get bottom flag
      drive(1000);
      // Turn off the flywheel
      rFlywheel(false);
      // Drive backward to align with the platform
      drive(2500, false);
      // Turn 90 degrees to face the platform
      turn(90, true);
      // Drive forward to get on the platform
      drive(1000);
      */
    break;
    // RED - RIGHT
    case 3:
    intakeMotor.move(127);
    indexerMotor.move(127);
    drive(1200);
    delay(1000);
    intakeMotor.move(0);
    indexerMotor.move(0);
    delay(500);
    drive(-200);
    delay(500);
    turn(45, true);

      /*
      // Run the intake
      intakeMotor.move(127);
      // Move toward cap to intake the ball
      drive(2000);
      // Wait 1/4 of a second to ensure we have the ball
      delay(250);
      // Stop the intake motor
      intakeMotor.move(0);
      // Drive backward to align with flags
      drive(1250, false);
      // Turn on the flywheel
      rFlywheel();
      // Turn 45 degrees to face the flags
      turn(45);
      // Turn on the intake to push balls up to the indexer
      intakeMotor.move(127);
      // Turn on the indexer to shoot the balls
      indexerMotor.move(127);
      // Delay by half a second to ensure the balls have been shot
      delay(500);
      // Stop the intake
      intakeMotor.move(0);
      // Stop the indexer
      indexerMotor.move(0);
      // Turn off the flywheel
      rFlywheel(false);
      // Turn 45 degrees to be parallel with the platform
      turn(45, true);
      // Slightly move forward to center with the platform
      drive(250);
      // Turn 90 degrees to face platform
      turn(90);
      // Move forward to get onto platform
      drive(1000);
      */
    break;
    // JASON COUNTER - BLUE
    case 4:
     // WORK ON LATER
    break;
    // JASON COUNTER - RED
    case 5:
      // WORK ON LATER
    break;
    // SKILLS
    case 6:
      // WORK ON LATER
    break;
    // DISABLE
    case 8:
      // DO NOTHING
    break;
  }
}
