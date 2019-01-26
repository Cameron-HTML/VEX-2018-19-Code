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

void drive(int power, int distance) {
  // Reset the motor encoder count
  leftFrontDriveMotor.tare_position();
  leftBackDriveMotor.tare_position();
  rightFrontDriveMotor.tare_position();
  rightBackDriveMotor.tare_position();

  // Init 'tickGoal'
  int tickGoal;

  // Calculate 'tickGoal'
  tickGoal = 29 * (distance - 6);

  while(abs(leftFrontDriveMotor.get_position()) < tickGoal) {
    if(abs(leftFrontDriveMotor.get_position()) < tickGoal - 10) {
      leftFrontDriveMotor.move(power);
      leftBackDriveMotor.move(power);
      rightFrontDriveMotor.move(power);
      rightBackDriveMotor.move(power);
    } else {
      if(power > 1) {
        leftFrontDriveMotor.move(5);
        leftBackDriveMotor.move(5);
        rightFrontDriveMotor.move(5);
        rightBackDriveMotor.move(5);
      } else {
        leftFrontDriveMotor.move(-5);
        leftBackDriveMotor.move(-5);
        rightFrontDriveMotor.move(-5);
        rightBackDriveMotor.move(-5);
      }
    }
    delay(20);
  }

  delay(100);
  leftFrontDriveMotor.move(0);
  leftBackDriveMotor.move(0);
  rightFrontDriveMotor.move(0);
  rightBackDriveMotor.move(0);
}

void turn(double time, bool leftTurn = true) {
  bool run = true

  while(run) {
    if(leftTurn) {
      leftFrontDriveMotor.move(-127);
      leftBackDriveMotor.move(-127);
      rightFrontDriveMotor.move(127);
      rightBackDriveMotor.move(127);
      delay(time);
      leftFrontDriveMotor.move(0);
      leftBackDriveMotor.move(0);
      rightFrontDriveMotor.move(0);
      rightBackDriveMotor.move(0);
      run = false;
    } else {
      leftFrontDriveMotor.move(127);
      leftBackDriveMotor.move(127);
      rightFrontDriveMotor.move(-127);
      rightBackDriveMotor.move(-127);
      delay(time);
      leftFrontDriveMotor.move(0);
      leftBackDriveMotor.move(0);
      rightFrontDriveMotor.move(0);
      rightBackDriveMotor.move(0);
      run = false;
    }
  }
}

void autonomous() {
  switch(mainContainer.LCD.auton) {
    // BLUE - LEFT
    case 0:
    mainContainer.flyWheel.PID.PIDRunning = true;
    delay(4000);
    intakeMotor.move(127);
    delay(1250);
    mainContainer.flyWheel.PID.PIDRunning = false;
    drive(-127, 6);
    delay(500);
    turn(DEG90);
    intakeMotor.move(127);
    drive(127, 38);
    delay(250);
    intakeMotor.move(0);
    drive(-127, 12);
    turn(DEG90, false);
    drive(127, 14);
    break;
    // BLUE - RIGHT
    case 1:
    /*
    intakeMotor.move(115);
    drive(110, 49);
    delay(225);
    intakeMotor.move(-60);
    delay(125);
    intakeMotor.move(0);
    mainContainer.flyWheel.PID.PIDRunning = true;
    delay(100);
    drive(-110, 53);
    drive(127, 8);
    turn(840, false);
    drive(60, 8);
    delay(4000);
    intakeMotor.move(127);
    delay(250);
    drive(115, 32);
    delay(100);
    intakeMotor.move(0);
    mainContainer.flyWheel.PID.PIDRunning = false;
    turn(260, false);
    drive(60, 20);
    drive(-127, 28);
    turn(900, false);
    drive(-110, 48);
    // mainContainer.twoBar.PID.requestedValue += 450;
    // mainContainer.flip.PID.PIDRunning = true;
    delay(20);
    // mainContainer.flip.PID.requestedValue += 180;
    drive(110, 14);
    turn(890, false);
    drive(127, 56);
    break;
    // RED - LEFT
    case 2:
    intakeMotor.move(127);
    drive(127, 50);
    delay(250);
    intakeMotor.move(-60);
    delay(125);
    intakeMotor.move(0);
    mainContainer.flyWheel.PID.PIDRunning = true;
    delay(100);
    drive(-127, 50);
    turn(790, true);
    drive(60, 8);
    delay(3000);
    intakeMotor.move(127);
    delay(250);
    drive(127, 32);
    delay(100);
    intakeMotor.move(0);
    mainContainer.flyWheel.PID.PIDRunning = false;
    turn(900, true);
    drive(60, 14);
    turn(870, false);
    drive(60, 20);
    drive(-127, 28);
    turn(900, true);
    drive(-110, 48);
    // mainContainer.twoBar.PID.requestedValue += 450;
    // mainContainer.flip.PID.PIDRunning = true;
    delay(20);
    // mainContainer.flip.PID.requestedValue += 180;
    drive(110, 14);
    turn(890, true);
    drive(127, 56);
    */
    break;
    // RED - LEFT
    case 2:
    intakeMotor.move(127);
    drive(127, 38);
    delay(300);
    intakeMotor.move(0);
    mainContainer.flyWheel.PID.PIDRunning = true;
    drive(-127, 35);
    turn(DEG90);
    intakeMotor.move(110);
    drive(127, 38);
    delay(250);
    drive(-127, 52);
    turn(DEG90, false);
    drive(127, 30);
    break;
    // RED - RIGHT
    case 3:
    mainContainer.flyWheel.PID.PIDRunning = true;
    delay(4000);
    intakeMotor.move(127);
    delay(1250);
    mainContainer.flyWheel.PID.PIDRunning = false;
    drive(-127, 6);
    delay(500);
    turn(DEG90, false);
    intakeMotor.move(127);
    drive(127, 38);
    delay(250);
    intakeMotor.move(0);
    drive(-127, 12);
    turn(DEG90);
    drive(127, 14);
    /*
    intakeMotor.move(127);
    drive(127, 50);
    drive(-127, 12);
    turn(890, false);
    drive(-127, 55);
    */
    break;
    // SKILLS
    case 4:
    break;
    // DISABLE
    case 5:
    break;
  }
}
