/**
 * \file main.h
 *
 * Contains common definitions and header files used throughout your PROS
 * project.
 *
 * Copyright (c) 2017-2018, Purdue University ACM SIGBots.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

/**
 * If defined, some commonly used enums will have preprocessor macros which give
 * a shorter, more convenient naming pattern. If this isn't desired, simply
 * comment the following line out.
 *
 * For instance, E_CONTROLLER_MASTER has a shorter name: CONTROLLER_MASTER.
 * E_CONTROLLER_MASTER is pedantically correct within the PROS styleguide, but
 * not convienent for most student programmers.
 */
#define PROS_USE_SIMPLE_NAMES

/**
 * If defined, C++ literals will be available for use. All literals are in the
 * pros::literals namespace.
 *
 * For instance, you can do `4_mtr = 50` to set motor 4's target velocity to 50
 */
#define PROS_USE_LITERALS

#include "api.h"
#include <vector>
#include <string> // std::stringstream .str()
#include <sstream> // std::stringstream

/**
 * You should add more #includes here
 */
//#include "okapi/api.hpp"
//#include "pros/api_legacy.h"

/**
 * If you find doing pros::Motor() to be tedious and you'd prefer just to do
 * Motor, you can use the namespace with the following commented out line.
 *
 * IMPORTANT: Only the okapi or pros namespace may be used, not both
 * concurrently! The okapi namespace will export all symbols inside the pros
 * namespace.
 */
using namespace pros;
using namespace pros::literals;
using namespace pros::lcd;
using namespace std;
// using namespace okapi;

inline string toString(float f){
  stringstream s;
  s << f;
  return s.str();
}

inline string toString(bool b){
  return (b ? "true" : "false");
  // stringstream s;
  // s << b;
  // return s.str();
}

inline string toString(int i){
  stringstream s;
  s << i;
  return s.str();
}

inline string toString(double d){
  stringstream s;
  s << d;
  return s.str();
}

inline string toString(float* f){
  return toString((float)(*f));
}

inline string toString(bool* b){
  return toString((bool)(*b));
}

inline string toString(int* i){
  return toString((int)(*i));
}

inline string toString(double* d){
  return toString((double)(*d));
}

inline float scaleData(float data, float minIn, float maxIn, float minOut, float maxOut){
  return ((data - minIn) / (maxIn - minIn)) * (maxOut - minOut) + minOut;
  // return ((data-min)/(max-min));
}

// Motors
inline Motor leftFrontDriveMotor(2, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);
inline Motor leftBackDriveMotor(7, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);
inline Motor rightFrontDriveMotor(3, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);
inline Motor rightBackDriveMotor(6, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);

inline Motor leftFlyWheelMotor(12, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);
inline Motor rightFlyWheelMotor(15, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);
inline Motor intakeMotor(9, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);
inline Motor indexerMotor(7, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);

// Gyro
inline ADIGyro gyro(1);

// Vision
inline Vision visionSensor(1);
inline vector<vision_object_s_t> objVec;

// Controllers
inline Controller master(E_CONTROLLER_MASTER);

// LCD structure
typedef struct LCD_D {
  vector<vector<string>> mainPages = {
    {"", "", "" , "" , "", "", "", ""},
    {"", "", "" , "" , "", "", "", ""},
    {"", "", "" , "" , "", "", "", ""},
    {"", "", "" , "" , "", "", "", ""}
  };

  vector<vector<string>> autoPages = {
    {"", "", "" , "" , "", "", "", ""},
    {"", "", "" , "" , "", "", "", ""},
    {"", "", "" , "" , "", "", "", ""},
    {"", "", "" , "" , "", "", "", ""},
    {"", "", "" , "" , "", "", "", ""},
    {"", "", "" , "" , "", "", "", ""},
  };

  int currentPage = 0;
  int auton = 0;
  bool autonSelected = false;
  bool autoSelection = false;
} _LCD;

// MAIN PID structure
typedef struct PID_d {
  // START OF PID VARIABLES
  float kP = 2.0;
  float kI = 0.04;
  float kD = 0.2;

  int encoderScale = 1;
  int motorScale = -1;
  int maxPower = 127;
  int minPower = -127;
  int integralLimit = 50;
  int requestedValueMax = 0;
  int requestedValueMin = 0;

  float requestedValue = 0.0;
  float currentSensorValue = 0.0;
  float error = 0.0;
  float lastError = 0.0;
  float integral = 0.0;
  float derivative = 0.0;
  float outputPower = 0.0;

  bool PIDRunning = true;
  bool limitRequestedValue = false;

  // END OF PID VARIABLES

  const int PID(double encoderPos) {
    // Check if the PID is running
    if(PIDRunning) {
      // Calculate the sensor value
      currentSensorValue = encoderPos * encoderScale;

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

      // Return final 'outputPower'
      return outputPower * motorScale;
    }
  }

  // PID reset
  const void PIDReset() {
    requestedValue = 0.0;
    currentSensorValue = 0.0;
    error = 0.0;
    lastError = 0.0;
    integral = 0.0;
    derivative = 0.0;
    outputPower = 0.0;
  }
} _PID;

// Flywheel structure
typedef struct flyWheel_d {
  _PID PID;

} _flyWheel;

/* Flipper structure
// Flipper structure
typedef struct flip_d {
  _PID PID;
} _flip;
*/

/* Two-bar structure
// Two-bar structure
typedef struct twoBar_d {
  _PID PID;
} _twoBar;
*/

/* Vision tracking struct
// Vision tracking struct
typedef struct vision_d {
  // 'trackingEnabled' variable
  bool trackingEnabled = false;

  int screenMiddleX = 316 / 4;

  int speed = 45;
} _vision;
*/

/* Drivetrain PID struct
// Drivetrain PID struct
typedef struct driveTrain_d {
  _PID PID;
} _driveTrain;
*/

// The 'container' that holds everything | hints the name 'mainContainer'
typedef struct mainContainer_d {
  _LCD LCD;
  _flyWheel flyWheel;
  // _flip flip;
  // _twoBar twoBar;
  // _vision vision;
} _mainContainer;

// Define the '_mainContainer' name
inline _mainContainer mainContainer;

// Flywheel PID task
inline void flyWheelPID(void* mainContainer_p) {
  _mainContainer *mainContainerPtr = (_mainContainer *)mainContainer_p;

  // Redefine the values needed
  mainContainerPtr->flyWheel.PID.kP = 2.0;
  mainContainerPtr->flyWheel.PID.kI = 0.04;
  mainContainerPtr->flyWheel.PID.kD = 0.2;
  mainContainerPtr->flyWheel.PID.PIDRunning = false;

  // Reset the value of the encoder
  leftFlyWheelMotor.tare_position();
  rightFlyWheelMotor.tare_position();

  while(true) {

    if(mainContainerPtr->flyWheel.PID.PIDRunning == true) {
      leftFlyWheelMotor.move(127);
      rightFlyWheelMotor.move(127);
    } else {
      // If the PID is off set the power to '0'
      leftFlyWheelMotor.move(0);
      rightFlyWheelMotor.move(0);
      leftFlyWheelMotor.tare_position();
      leftFlyWheelMotor.tare_position();

      // Reset the values aswell
      mainContainerPtr->flyWheel.PID.PIDReset();
    }
    delay(20);
  }
}

/* Flipper PID task
// Flipper PID task
inline void flipPID(void* mainContainer_p) {
  _mainContainer *mainContainerPtr = (_mainContainer *)mainContainer_p;

  // Redefine the values needed
  mainContainerPtr->flip.PID.kP = 1.75;
  mainContainerPtr->flip.PID.kI = 0.04;
  mainContainerPtr->flip.PID.kD = 0.2;
  mainContainerPtr->flip.PID.maxPower = 115;
  mainContainerPtr->flip.PID.minPower = -115;
  mainContainerPtr->flip.PID.PIDRunning = false;

  // Reset the value of the encoder
  flipperMotor.tare_position();

  // Enter loop
  while(true) {
    if(mainContainerPtr->flip.PID.PIDRunning) {
      // If the PID is on set the motor power to 'outputPower'
      flipperMotor.move(mainContainerPtr->flip.PID.PID(flipperMotor.get_position()));
    } else {
      // If the PID is off reset the values
      mainContainerPtr->flip.PID.PIDReset();
    }
    delay(20);
  }
}
*/

/* Tracking Task
// Tracking task
inline void tracking(void* mainContainer_p) {
  _mainContainer *mainContainerPtr = (_mainContainer *)mainContainer_p;
  while(true) {

    delay(20);
  }
}
*/

/* Two-bar PID task
// Two-bar PID task
inline void twoBarPID(void* mainContainer_p) {
  _mainContainer *mainContainerPtr = (_mainContainer *)mainContainer_p;

  // Redefine needed values
  mainContainerPtr->twoBar.PID.kP = 0.6;
  mainContainerPtr->twoBar.PID.kI = 0.04;
  mainContainerPtr->twoBar.PID.kD = 0.2;
  mainContainerPtr->twoBar.PID.PIDRunning = true;
  mainContainerPtr->twoBar.PID.limitRequestedValue = true;
  mainContainerPtr->twoBar.PID.requestedValueMax = 3050;

  // Reset the value of the encoder
  twoBarMotor.tare_position();

  // Enter loop
  while(true) {
    // Check if the PID is running
    if(mainContainerPtr->twoBar.PID.PIDRunning) {
      // If the PID is on set the motor power to the 'outputPower'
      twoBarMotor.move(mainContainerPtr->twoBar.PID.PID(twoBarMotor.get_position()));
    } else {
      // If the PID is off reset the values
      mainContainerPtr->twoBar.PID.PIDReset();
    }
    delay(20);
  }
}
*/

// LCD task
inline void LCDUpdate(void* mainContainer_p) {
  _mainContainer *mainContainerPtr = (_mainContainer *)mainContainer_p;

	while(true) {
		if(mainContainerPtr->LCD.autoSelection && !mainContainerPtr->LCD.autonSelected) {

			// Auto pages
			mainContainerPtr->LCD.autoPages[0][3] = "BLUE - LEFT";
			mainContainerPtr->LCD.autoPages[0][4] = "<Select>";
			mainContainerPtr->LCD.autoPages[1][3] = "BLUE - RIGHT";
			mainContainerPtr->LCD.autoPages[1][4] = "<Select>";
			mainContainerPtr->LCD.autoPages[2][3] = "RED - LEFT";
			mainContainerPtr->LCD.autoPages[2][4] = "<Select>";
			mainContainerPtr->LCD.autoPages[3][3] = "RED - RIGHT";
			mainContainerPtr->LCD.autoPages[3][4] = "<Select>";
      mainContainerPtr->LCD.autoPages[4][3] = "SKILLS";
			mainContainerPtr->LCD.autoPages[4][4] = "<Select>";
			mainContainerPtr->LCD.autoPages[5][3] = "DISABLE";
			mainContainerPtr->LCD.autoPages[5][4] = "<Select>";

			if(read_buttons() == LCD_BTN_LEFT) {
				mainContainerPtr->LCD.currentPage -= 1;

				if(mainContainerPtr->LCD.currentPage < 0) {
					mainContainerPtr->LCD.currentPage = mainContainerPtr->LCD.autoPages.size() - 1;
				}

				while(read_buttons() == LCD_BTN_LEFT) delay(20);
			}
			else if(read_buttons() == LCD_BTN_RIGHT) {
				mainContainerPtr->LCD.currentPage += 1;

				if(mainContainerPtr->LCD.currentPage > mainContainerPtr->LCD.autoPages.size() - 1) {
					mainContainerPtr->LCD.currentPage = 0;
				}

				while(read_buttons() == LCD_BTN_RIGHT) delay(20);
			}
			else if(read_buttons() == LCD_BTN_CENTER) {

				mainContainerPtr->LCD.auton = mainContainerPtr->LCD.currentPage;
				mainContainerPtr->LCD.currentPage = 0;
				mainContainerPtr->LCD.autonSelected = true;

				while(read_buttons() == LCD_BTN_CENTER) delay(20);
			}



			//Update The Screen
			for (unsigned i = 0; i < mainContainerPtr->LCD.autoPages[mainContainerPtr->LCD.currentPage].size(); i++) {
				set_text(i, mainContainerPtr->LCD.autoPages[mainContainerPtr->LCD.currentPage][i]);
			}
		}
		else {

			mainContainerPtr->LCD.mainPages[0][0] = "Drivetrain motors:";
		  mainContainerPtr->LCD.mainPages[0][1] = "Front left: " + toString(gyro.get_value());
      mainContainerPtr->LCD.mainPages[0][2] = "Back left: " + toString(leftBackDriveMotor.get_temperature());
			mainContainerPtr->LCD.mainPages[0][3] = "Front right: " + toString(rightFrontDriveMotor.get_temperature());
			mainContainerPtr->LCD.mainPages[0][4] = "Back right: " + toString(rightBackDriveMotor.get_temperature());
			mainContainerPtr->LCD.mainPages[0][7] = "Motor temperatures - <1/2>";

			mainContainerPtr->LCD.mainPages[1][0] = "Flywheel motors:";
			mainContainerPtr->LCD.mainPages[1][1] = "Flywheel: " + toString(leftFlyWheelMotor.get_temperature());
      mainContainerPtr->LCD.mainPages[1][1] = "Flywheel: " + toString(rightFlyWheelMotor.get_temperature());
			mainContainerPtr->LCD.mainPages[1][2] = "Intake: " + toString(intakeMotor.get_temperature());
			mainContainerPtr->LCD.mainPages[1][7] = "Motor temperatures - <2/2>";

			mainContainerPtr->LCD.mainPages[2][0] = "Drivetrain motors:";
			mainContainerPtr->LCD.mainPages[2][1] = "Front left: " + toString(leftFrontDriveMotor.get_efficiency());
			mainContainerPtr->LCD.mainPages[2][2] = "Back left: " + toString(leftBackDriveMotor.get_efficiency());
			mainContainerPtr->LCD.mainPages[2][3] = "Front right: " + toString(rightFrontDriveMotor.get_efficiency());
			mainContainerPtr->LCD.mainPages[2][4] = "Back right: " + toString(rightBackDriveMotor.get_efficiency());
			mainContainerPtr->LCD.mainPages[2][7] = "Motor efficiency - <1/2>";

			mainContainerPtr->LCD.mainPages[3][0] = "Flywheel motors:";
			mainContainerPtr->LCD.mainPages[3][1] = "Flywheel: " + toString(leftFlyWheelMotor.get_efficiency());
      mainContainerPtr->LCD.mainPages[3][2] = "Flywheel: " + toString(rightFlyWheelMotor.get_efficiency());
			mainContainerPtr->LCD.mainPages[3][3] = "Intake: " + toString(intakeMotor.get_efficiency());
			mainContainerPtr->LCD.mainPages[3][7] = "Motor efficiency - <2/2>";

			if(read_buttons() == LCD_BTN_LEFT) {
				mainContainerPtr->LCD.currentPage -= 1;

				if(mainContainerPtr->LCD.currentPage < 0) {
					mainContainerPtr->LCD.currentPage = mainContainerPtr->LCD.mainPages.size() - 1;
				}

				while(read_buttons() == LCD_BTN_LEFT) delay(20);
			}
			else if(read_buttons() == LCD_BTN_RIGHT) {
				mainContainerPtr->LCD.currentPage += 1;

				if(mainContainerPtr->LCD.currentPage > mainContainerPtr->LCD.mainPages.size() - 1) {
					mainContainerPtr->LCD.currentPage = 0;
				}

				while(read_buttons() == LCD_BTN_RIGHT) delay(20);
			}
			else if(read_buttons() == LCD_BTN_CENTER) {
				mainContainerPtr->LCD.currentPage = 0;

				while(read_buttons() == LCD_BTN_CENTER) delay(20);
			}

			//Update The Screen
			for (unsigned i = 0; i < mainContainerPtr->LCD.mainPages[mainContainerPtr->LCD.currentPage].size(); i++) {
				set_text(i, mainContainerPtr->LCD.mainPages[mainContainerPtr->LCD.currentPage][i]);
			}
		}
		delay(20);
	}
}

/**
 * Prototypes for the competition control tasks are redefined here to ensure
 * that they can be called from user code (i.e. calling autonomous from a
 * button press in opcontrol() for testing purposes).
 */
#ifdef __cplusplus
extern "C" {
#endif
void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/**
 * You can add C++-only headers here
 */
//#include <iostream>
#endif

#endif  // _PROS_MAIN_H_
