// Copyright 1996-2024 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 Combined version:
 - Robot walks forward automatically
 - Arrow keys allow turning / side-stepping
*/

#include "Walk.hpp"

#include <RobotisOp2GaitManager.hpp>
#include <RobotisOp2MotionManager.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/Keyboard.hpp>
#include <webots/LED.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>

using namespace webots;
using namespace managers;
using namespace std;

static const char *motorNames[NMOTORS] = {
  "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */, "ArmLowerR" /*ID5 */,
  "ArmLowerL" /ID6 */, "PelvYR" /*ID7 */,    "PelvYL" /*ID8 */,    "PelvR" /*ID9 */,     "PelvL" /*ID10/,
  "LegUpperR" /ID11/, "LegUpperL" /ID12/, "LegLowerR" /ID13/, "LegLowerL" /ID14/, "AnkleR" /ID15/,
  "AnkleL" /ID16/,    "FootR" /ID17/,     "FootL" /ID18/,     "Neck" /ID19/,      "Head" /ID20/
};

Walk::Walk() : Robot() {
  mTimeStep = getBasicTimeStep();

  getLED("HeadLed")->set(0xFF0000);
  getLED("EyeLed")->set(0x00FF00);
  mAccelerometer = getAccelerometer("Accelerometer");
  mAccelerometer->enable(mTimeStep);

  getGyro("Gyro")->enable(mTimeStep);

  for (int i = 0; i < NMOTORS; i++) {
    mMotors[i] = getMotor(motorNames[i]);
    string sensorName = motorNames[i];
    sensorName.push_back('S');
    mPositionSensors[i] = getPositionSensor(sensorName);
    mPositionSensors[i]->enable(mTimeStep);
  }

  mKeyboard = getKeyboard();
  mKeyboard->enable(mTimeStep);

  mMotionManager = new RobotisOp2MotionManager(this);
  mGaitManager = new RobotisOp2GaitManager(this, "config.ini");
}

Walk::~Walk() {}

void Walk::myStep() {
  int ret = step(mTimeStep);
  if (ret == -1)
    exit(EXIT_SUCCESS);
}

void Walk::wait(int ms) {
  double startTime = getTime();
  double s = (double)ms / 1000.0;
  while (s + startTime >= getTime())
    myStep();
}

// Combined run(): Auto forward + keyboard control
void Walk::run() {
  cout << "=======================================" << endl;
  cout << "  ROBOTIS OP2 - AUTO FORWARD WALKING  " << endl;
  cout << "=======================================" << endl;
  cout << "Robot will walk forward automatically" << endl;
  cout << "Use arrow keys to turn or side-step while walking" << endl;
  cout << "Press Ctrl+C to stop the simulation" << endl;
  cout << endl;

  // First step to update sensors values
  myStep();

  // Initialize pose
  mMotionManager->playPage(9);  // init position
  wait(200);

  // Start gait
  mGaitManager->start();
  wait(200);

  const double WALK_SPEED = 1.0;  // Constant forward speed
  double turnAngle = 0.0;         // Turning amplitude
  double sideStep = 0.0;          // Side stepping amplitude

  cout << "Robot is now walking forward automatically!" << endl;

  // Main control loop
  while (true) {
    checkIfFallen();

    // Base forward walking
    mGaitManager->setXAmplitude(WALK_SPEED);

    // Default: go straight
    turnAngle = 0.0;
    sideStep = 0.0;

    // Check keyboard input
    int key = mKeyboard->getKey();
    if (key != -1) {
      switch (key) {
        case Keyboard::LEFT:
          turnAngle = 0.5;   // turn left
          break;
        case Keyboard::RIGHT:
          turnAngle = -0.5;  // turn right
          break;
        case Keyboard::UP:
          sideStep = 0.2;    // side-step right
          break;
        case Keyboard::DOWN:
          sideStep = -0.2;   // side-step left
          break;
      }
    }

    // Apply movement parameters
    mGaitManager->setAAmplitude(turnAngle);
    mGaitManager->setYAmplitude(sideStep);

    // Step gait and simulation
    mGaitManager->step(mTimeStep);
    myStep();

    // Print status occasionally
    static double lastPrint = 0;
    if (getTime() - lastPrint > 5.0) {
      cout << "Time: " << getTime() << "s | Turning: " << turnAngle
           << " | Side-step: " << sideStep << endl;
      lastPrint = getTime();
    }
  }
}

// Fall recovery (same as original)
void Walk::checkIfFallen() {
  static int fup = 0;
  static int fdown = 0;
  static const double acc_tolerance = 80.0;
  static const double acc_step = 100;

  const double *acc = mAccelerometer->getValues();
  if (acc[1] < 512.0 - acc_tolerance)
    fup++;
  else
    fup = 0;

  if (acc[1] > 512.0 + acc_tolerance)
    fdown++;
  else
    fdown = 0;

  if (fup > acc_step) {
    cout << "Robot fell forward! Getting back up..." << endl;
    mMotionManager->playPage(10);  // f_up
    mMotionManager->playPage(9);   // init position
    mGaitManager->start();
    fup = 0;
  } else if (fdown > acc_step) {
    cout << "Robot fell backward! Getting back up..." << endl;
    mMotionManager->playPage(11);  // b_up
    mMotionManager->playPage(9);   // init position
    mGaitManager->start();
    fdown = 0;
  }
}