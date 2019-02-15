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

  while(mainContainer.driveTrain.PID.PIDRunning) delay(10);

  leftFrontDriveMotor.move(0);
  leftBackDriveMotor.move(0);
  rightFrontDriveMotor.move(0);
  rightBackDriveMotor.move(0);
}

void turn(int degrees, bool right = false) {
  leftFrontDriveMotor.tare_position();

  while(abs(leftFrontDriveMotor.get_position()) < degrees) {
    if(!right) {
      if(abs(leftFrontDriveMotor.get_position()) < (degrees - 60)) {
        leftFrontDriveMotor.move(-90);
        leftBackDriveMotor.move(-90);
        rightFrontDriveMotor.move(90);
        rightBackDriveMotor.move(90);
      } else {
        leftFrontDriveMotor.move(-10);
        leftBackDriveMotor.move(-10);
        rightFrontDriveMotor.move(10);
        rightBackDriveMotor.move(10);
      }
    } else {
      if(abs(leftFrontDriveMotor.get_position()) < (degrees - 60)) {
        leftFrontDriveMotor.move(90);
        leftBackDriveMotor.move(90);
        rightFrontDriveMotor.move(-90);
        rightBackDriveMotor.move(-90);
      } else {
        leftFrontDriveMotor.move(10);
        leftBackDriveMotor.move(10);
        rightFrontDriveMotor.move(-10);
        rightBackDriveMotor.move(-10);
      }
    }
    delay(40);
  }
  leftFrontDriveMotor.move(0);
  leftBackDriveMotor.move(0);
  rightFrontDriveMotor.move(0);
  rightBackDriveMotor.move(0);
}

void rFlywheel(bool run = true) {
  mainContainer.flyWheel.PID.PIDRunning = run;
}

/* REMEMBER TO TEST ALL OF THE BELOW */

void autonomous() {
  switch(mainContainer.LCD.auton) {
    // BLUE - LEFT
    case 0:
      turn(320);
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

    intakeMotor.move(127);
    indexerMotor.move(72);
    rFlywheel();
    drive(1300);
    delay(250);
    intakeMotor.move(0);
    indexerMotor.move(0);
    drive(-1500);
    drive(100);
    turn(270);
    drive(275);
    intakeMotor.move(127);
    indexerMotor.move(127);
    delay(600);
    leftFrontDriveMotor.move(96);
    leftBackDriveMotor.move(96);
    rightFrontDriveMotor.move(127);
    rightBackDriveMotor.move(127);
    delay(1500);
    rFlywheel(false);
    indexerMotor.move(40);
    leftFrontDriveMotor.move(0);
    leftBackDriveMotor.move(0);
    rightFrontDriveMotor.move(0);
    rightBackDriveMotor.move(0);
    delay(500);
    leftFrontDriveMotor.move(-127);
    leftBackDriveMotor.move(-127);
    rightFrontDriveMotor.move(-100);
    rightBackDriveMotor.move(-100);
    delay(600);
    leftFrontDriveMotor.move(0);
    leftBackDriveMotor.move(0);
    rightFrontDriveMotor.move(0);
    rightBackDriveMotor.move(0);
    turn(320, true);
    delay(300);
    leftFrontDriveMotor.move(-127);
    leftBackDriveMotor.move(-127);
    rightFrontDriveMotor.move(-127);
    rightBackDriveMotor.move(-127);
    delay(750);
    indexerMotor.move(40);
    leftFrontDriveMotor.move(0);
    leftBackDriveMotor.move(0);
    rightFrontDriveMotor.move(0);
    rightBackDriveMotor.move(0);
    intakeMotor.move(-70);
    drive(1225);
    turn(330, true);
    intakeMotor.move(127);
    drive(1600);

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
      rFlywheel();
      intakeMotor.move(127);
      indexerMotor.move(60);
      drive(1200);
      delay(500);
      intakeMotor.move(0);
      indexerMotor.move(0);
      drive(-400);
      turn(320, true);
      drive(550);
      turn(320);
      intakeMotor.move(-127);
      drive(600);
      drive(-600);
      turn(330, true);
      drive(-500);
      turn(320);
      drive(-800);
      /*
      drive(-400);
      turn(340, true);
      drive(-500);
      turn(340);
      drive(-1300);
      */

      /*
      delay(250);
      indexerMotor.move(127);
      intakeMotor.move(127);
      delay(500);
      indexerMotor.move(0);
      intakeMotor.move(0);
      delay(500);
      drive(400);
      turn(300, true);
      delay(250);
      drive(-300);
      turn(320);
      drive(800);
      */
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
    // SKILLS
    case 4:
      // WORK ON LATER
    break;
    // DISABLE
    case 5:
      // DO NOTHING
    break;
  }
}
