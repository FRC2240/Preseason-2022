// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {
  bool toggleE = false;
}

void Robot::TeleopPeriodic() {
  //read joystick controls
  // I hate you, Ethan
  double strafeRight = m_stick.GetRawAxis(2); //Possibly Reversed
  double backForth = m_stick.GetRawAxis(1);
  double rotate = m_stick.GetRawAxis(3); //Possibly reversed
  //double Throttle = m_stick.GetRawAxis(3); //Change to whatever the Z axis is
  double redMode = m_stick.GetRawButtonPressed(25);
  double yellowMode = m_stick.GetRawButtonPressed(24);
  double greenMode = m_stick.GetRawButtonPressed(23);

  double climbButton = m_stick.GetRawButtonPressed(1); //Fire!
  double holdTrigger = m_stick.GetRawButtonPressed(0);
  double lowTrigger = m_stick.GetRawButtonPressed(5);
  double buttonA = m_stick.GetRawButtonPressed(2);
  double buttonB = m_stick.GetRawButtonPressed(3);
  double buttonC = m_stick.GetRawButtonPressed(4);
  double buttonE = m_stick.GetRawButtonPressed(7);
  double speedMod;

  if (redMode) {
    SpeedMod = 3;
  }
  else if (yellowMode) {
    speedMod = 1;
  }

  else if (greenMode) {
    speedMod = 0.5;
  }

  else if (buttonE) {
    std::cout << "E";
  }

  m_robotDrive.driveCartesian(backForth*speedMod, -strafeRight, rotate); //https://docs.wpilib.org/en/stable/docs/software/actuators/wpi-drive-classes.html

  if (climbButton) {
    m_leftClimbMotor.Set(0.25);
    m_rightClimbMotor.Set(0.25); //don't know if this will work

  }

  if (holdTrigger) {
    m_armMotor.Set(0.1);
  }

  if (!holdTrigger) {
    m_armMotor.Set(-0.1);
  }

  if (lowTrigger) {
    m_grabberMotor.Set(frc::DoubleSolenoid::Value::kForward);
  }

  if (!lowTrigger) {
    m_grabberMotor.Set(frc::DoubleSolenoid::Value::kReverse);
  }




}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif