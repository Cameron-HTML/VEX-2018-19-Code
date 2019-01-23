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

/* OLD FUNCTIONS
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
    leftFrontDriveMotor.move(power);
    leftBackDriveMotor.move(power);
    rightFrontDriveMotor.move(power);
    rightBackDriveMotor.move(power);

    delay(20);
  }

  if(power > 1) {
    leftFrontDriveMotor.move(-5);
    leftBackDriveMotor.move(-5);
    rightFrontDriveMotor.move(-5);
    rightBackDriveMotor.move(-5);
  } else {
    leftFrontDriveMotor.move(5);
    leftBackDriveMotor.move(5);
    rightFrontDriveMotor.move(5);
    rightBackDriveMotor.move(5);
  }
  delay(100);
  leftFrontDriveMotor.move(0);
  leftBackDriveMotor.move(0);
  rightFrontDriveMotor.move(0);
  rightBackDriveMotor.move(0);
}

void turn(int degrees, bool left) {
  // Reset the gyro value
  gyro.reset();

  if(left) {
    while(abs(gyro.get_value()) < degrees - 90) {
      leftFrontDriveMotor.move(80);
      leftBackDriveMotor.move(80);
      rightFrontDriveMotor.move(-80);
      rightBackDriveMotor.move(-80);

      delay(20);
    }

    leftFrontDriveMotor.move(-20);
    leftBackDriveMotor.move(-20);
    rightFrontDriveMotor.move(20);
    rightBackDriveMotor.move(20);
    delay(100);

    leftFrontDriveMotor.move(0);
    leftBackDriveMotor.move(0);
    rightFrontDriveMotor.move(0);
    rightBackDriveMotor.move(0);
  } else {
    while(abs(gyro.get_value()) < degrees - 90) {
      leftFrontDriveMotor.move(-80);
      leftBackDriveMotor.move(-80);
      rightFrontDriveMotor.move(80);
      rightBackDriveMotor.move(80);
      delay(20);
    }

    leftFrontDriveMotor.move(20);
    leftBackDriveMotor.move(20);
    rightFrontDriveMotor.move(-20);
    rightBackDriveMotor.move(-20);
    delay(100);

    leftFrontDriveMotor.move(0);
    leftBackDriveMotor.move(0);
    rightFrontDriveMotor.move(0);
    rightBackDriveMotor.move(0);
  }
}
*/

// PID FUNCTION
void DTPID(float requestedValue, bool useEncoder) {
  // START OF PID VARIABLES
  float kP = 1.0;
  float kI = 0.0;
  float kD = 0.0;

  int encoderScale = 1;
  int motorScale = -1;
  int maxPower = 127;
  int minPower = -127;
  int integralLimit = 50;
  int requestedValueMax = 0;
  int requestedValueMin = 0;

  float currentSensorValue = 0.0;
  float error = 0.0;
  float lastError = 0.0;
  float integral = 0.0;
  float derivative = 0.0;
  float outputPower = 0.0;

  bool PIDRunning = true;
  bool limitRequestedValue = false;

  while(currentSensorValue != requestedValue) {
    // Calculate the sensor value
    if(useEncoder) {
      currentSensorValue = leftFrontDriveMotor.get_position() * encoderScale;
    } else {
      currentSensorValue = gyro.get_value() * encoderScale;
    }

    // Limit the requested 'requestedValue'
    if(limitRequestedValue) {
      if(requestedValue > requestedValueMax) {
        requestedValue = requestedValueMax;
      } else if(requestedValue < requestedValueMin) {
        requestedValue = requestedValueMin;
      }
    }

    // Calculate error
    error = currentSensorValue - requestedValue;

    // Check if 'kI' is not equal to '0'
    if(kI != 0) {
      if(abs(error) < integralLimit) {
        integral = integral + error;
      } else {
        integral = 0;
      }
    } else {
      integral = 0;
    }

    // Calculate 'derivative'
    derivative = error - lastError;

    // Update 'lastError'
    lastError = error;

    // Calculate 'outputPower'
    outputPower = (kP * error) + (kI * integral) + (kD * derivative);

    // Limit 'outputPower'
    if(outputPower > maxPower) {
      outputPower = maxPower;
    } else if(outputPower < minPower) {
      outputPower = minPower;
    }
  }
}

void autonomous() {
  switch(mainContainer.LCD.auton) {
    // BLUE - LEFT
    case 0:
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
    // RED - RIGHT
    case 3:
    /*
    intakeMotor.move(127);
    drive(127, 50);
    drive(-127, 12);
    turn(890, false);
    drive(-127, 55);
    */
    break;
    // DISABLE
    case 5:
    break;
  }
}
