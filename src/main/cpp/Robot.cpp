// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  /* no idea what's supposed to go in here
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  */
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
  /*
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
  */
}

void Robot::AutonomousPeriodic() {
  autoTimer.Start();
  if (autoTimer.Get() <= 0.25) {
    m_robotDrive.DriveCartesian(0.5,0,0);
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {
  
}

void Robot::TeleopPeriodic() {
  //read joystick controls
  // I hate you, Ethan
  double fighterY = m_stick.GetRawAxis(2); //Possibly Reversed
  double fighterX = m_stick.GetRawAxis(1);
  double fighterZ = m_stick.GetRawAxis(3); //Possibly reversed
  //double Throttle = m_stick.GetRawAxis(3); //Change to whatever the Z axis is
  double redMode = m_stick.GetRawButtonPressed(25);         // hyperspeed and fun
  double yellowMode = m_stick.GetRawButtonPressed(24);
  double greenMode = m_stick.GetRawButtonPressed(23);       // slow and boring

  double climbButton = m_stick.GetRawButtonPressed(1); //Fire!
  double holdTrigger = m_stick.GetRawButtonPressed(0); //Misnomer. You don't need to hold the button
  double lowTrigger = m_stick.GetRawButtonPressed(5);
  double buttonA = m_stick.GetRawButtonPressed(2);
  double buttonB = m_stick.GetRawButtonPressed(3);
  double buttonC = m_stick.GetRawButtonPressed(4);
  double buttonE = m_stick.GetRawButtonPressed(7);
  double speedMod;

  std::cout << m_armEncoder.GetPosition() * 360; //Testing for the arm
  bool lowTriggerToggle;
  bool triggerToggle;

  if (redMode) {
    speedMod = 1;
  }
  else if (yellowMode) {
    speedMod = 0.66;
  }

  if (greenMode) {
    speedMod = 0.33;
  }

  if (buttonE) {
    std::cout << "E";
  }

  m_robotDrive.DriveCartesian(fighterX*speedMod, -fighterY*speedMod, fighterZ*speedMod); //https://docs.wpilib.org/en/stable/docs/software/actuators/wpi-drive-classes.html

  if (climbButton && m_climbEncoder.GetVelocity() < 500) {
    m_climbMotor.Set(0.25); //don't know if this will work



  }

  if (holdTrigger) {
    //This raises/lowers the arm for putting on the gear

      if (m_armEncoder.GetPosition() *360 <= 90 ) { // 90 is a placeholder for a thing
        // 
        m_armMotor.Set(0.1);
        m_grabberMotor.set(-0.1);
          // It's all placeholders 
        //Put the rotation of the wrist here
}
        else {           // Thank you Tyler from WPILib, your advice is much appreciated (@calcmogul#3301)
            m_armMotor.Set(0.0);
            //Put the rotation of the wrist here
  }
}

    //I don't know where default is. This is highly experimental.
  if (!holdTrigger) {
    if (m_armEncoder.GetPosition()*360 <= 35 ) {                // 35 is arbitrary. I'll do the math later 
    m_armMotor.Set(-0.1);
    m_grabberMotor.Set(-0.01);

    if (m_grabberMotor.GetPosition()*360 <= 30) {
      m_grabberMotor.Set(0.01);
    }
  }

  if (lowTrigger) {                                         // Low Trigger toggles the piston
      if (!lowTriggerToggle);
        m_grabberPiston.Set(frc::DoubleSolenoid::Value::kForward);
        lowTriggerToggle = true;
  }
    else {
        m_grabberPiston.Set(frc::DoubleSolenoid::Value::kReverse);
        lowTrigger = false;
  }

}
//bifle
  }

// https://cynosure.neocities.org/topsneaky.html

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
