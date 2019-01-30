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

void drive(int distance) {
  // Reset the motor encoder count
  leftFrontDriveMotor.tare_position();

  // Init variables
  int currentSensorValueL = 0;
  int currentSensorValueR = 0;
  int totalTicks = 0;
  int outputPowerL = 0;
  int outputPowerR = 0;
  int minPower = -127;
  int maxPower = 127;
  float tickGoal = 0;
  int errorL = 0;
  int errorR = 0;
  int kP = 1;

  while(abs(totalTicks) < distance) {
    // Calculate the sensor value for left and right
    currentSensorValueL = leftFrontDriveMotor.get_position();
    currentSensorValueR = rightFrontDriveMotor.get_position();

    errorL = currentSensorValueL - tickGoal;
    errorR = currentSensorValueR - tickGoal;

    outputPowerL += errorL / kP;
    outputPowerR += errorR / kP;

    // Limit 'outputPower'
    if(outputPowerL > maxPower) {
      outputPowerL = maxPower;
    } else if(outputPowerL < minPower) {
      outputPowerL = minPower;
    }

    if(outputPowerR > maxPower) {
      outputPowerR = maxPower;
    } else if(outputPowerR < minPower) {
      outputPowerR = minPower;
    }

    leftFrontDriveMotor.move(outputPowerL);
    leftBackDriveMotor.move(outputPowerL);
    rightFrontDriveMotor.move(outputPowerR);
    rightBackDriveMotor.move(outputPowerR);

    totalTicks += leftFrontDriveMotor.get_position();

    delay(20);
  }
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
}

void autonomous() {
  switch(mainContainer.LCD.auton) {
    // BLUE - LEFT
    case 0:
    intakeMotor.move(127);
    indexerMotor.move(127);
    drive(105, 34);
    delay(500);
    drive(-105, 13);
    delay(250);
    turn(DEG90, false);
    delay(DEG90);
    drive(-127, 5);
    drive(127, 30);
    /*
    mainContainer.flyWheel.PID.PIDRunning = true;
    delay(3500);
    intakeMotor.move(127);
    indexerMotor.move(127);
    delay(1000);
    mainContainer.flyWheel.PID.PIDRunning = false;
    delay(500);
    turn(DEG90);
    delay(DEG90);
    intakeMotor.move(127);
    indexerMotor.move(127);
    drive(127, 38);
    delay(250);
    intakeMotor.move(0);
    indexerMotor.move(0);
    drive(-127, 15);
    turn(DEG90, false);
    delay(DEG90);
    drive(127, 14);
    */
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
    drive(127, 40);
    delay(300);
    intakeMotor.move(0);
    mainContainer.flyWheel.PID.PIDRunning = true;
    drive(-127, 50);
    drive(127, 4);
    delay(250);
    turn(DEG90);
    delay(DEG90);
    intakeMotor.move(110);
    indexerMotor.move(110);
    drive(127, 38);
    delay(250);
    drive(-127, 52);
    turn(DEG90, false);
    delay(DEG90);
    drive(127, 30);
    break;
    // RED - RIGHT
    case 3:
    mainContainer.flyWheel.PID.PIDRunning = true;
    delay(4000);
    intakeMotor.move(127);
    indexerMotor.move(127);
    delay(1250);
    mainContainer.flyWheel.PID.PIDRunning = false;
    drive(-127, 6);
    delay(500);
    turn(DEG90, false);
    delay(DEG90);
    intakeMotor.move(127);
    indexerMotor.move(127);
    drive(127, 40);
    delay(250);
    intakeMotor.move(0);
    indexerMotor.move(0);
    drive(-127, 15);
    turn(DEG90);
    delay(DEG90);
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
