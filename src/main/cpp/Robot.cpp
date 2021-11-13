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
  double driveY;
  double driveX;
  double driveZ; 
  double climbButton;
  double armButtonDeploy;
  double armButtonReturn;
  double grabberButton;
  
    //void fighterMode() { 
      // function to read controls for the joystick
        driveY = m_stick.GetRawAxis(0);                    //Possibly Reversed
        driveX = m_stick.GetRawAxis(1);
        driveZ = m_stick.GetRawAxis(5); //Possibly reversed
        //double Throttle = m_stick.GetRawAxis(3); //Change to whatever the Z axis is

//all button bindings need to be tested
        climbButton = m_stick.GetRawButtonPressed(2); //Fire!
        armButtonDeploy = m_stick.GetRawButtonPressed(1); //Misnomer. You don't need to hold the button
        armButtonReturn = m_stick.GetRawButtonReleased(2); // don't know
        grabberButton = m_stick.GetRawButtonPressed(5);
        double buttonA = m_stick.GetRawButtonPressed(2);
        double buttonB = m_stick.GetRawButtonPressed(3);
        double buttonC = m_stick.GetRawButtonPressed(4);
        double buttonE = m_stick.GetRawButtonPressed(7);
        
/*
  void xboxMode() {

    driveY = m_stick.GetRawAxis(2);
    driveX = m_stick.GetRawAxis(1);
    driveZ = m_stick.GetRawAxis(4);

    double climbButton = m_stick.GetRawButtonPressed(9);
    double armButton = m_stick.GetRawButtonPressed(6);
    double grabberButton = m_stick.GetRawButtonPressed(5);
    double midSpeed = m_stick.GetRawAxis(6);
    if (midSpeed > 0.25) {
      midSpeed = 1;
    }
    double lowSpeed = m_stick.GetRawAxis(3);
    if (lowSpeed > 0.25) {
      lowSpeed = 1;
    }
  }

  xboxMode(); //change to fighterMode() if the joystick works
*/
  std::cout << m_armEncoder.GetPosition() * 360; //Testing for the arm
  bool lowTriggerToggle;
  bool triggerToggle;

  m_robotDrive.DriveCartesian(-driveY, driveX, -driveZ); //https://docs.wpilib.org/en/stable/docs/software/actuators/wpi-drive-classes.html
  

  if (climbButton && m_climbEncoder.GetVelocity() < 500) {
    m_climbMotor.Set(0.25); //don't know if this will work
  }
  
  //soft position value placeholders
  double x; //grabber rotation- soft position 1
  double v; //arm vertical - soft position 2
  double y; //grabber rotation - soft position 3
  double z; //arm deployed - final position


  if (armButtonDeploy) {
    //This raises/lowers the arm for putting on the gear

    //grabber rotates to soft position 1
      if (m_armEncoder.GetPosition() == 0 && m_grabberEncoder.GetPosition() < x ) {
        m_armMotor.Set(0);
        m_grabberMotor.Set(0.25);
      }

      //arm moves up to vertical to soft position 2
      else if (m_armEncoder.GetPosition() < v && m_grabberEncoder.GetPosition() == x) {
        m_grabberMotor.Set(0);
        m_armMotor.Set(0.25);
      }

      //grabber rotates backwards to soft position 3 (movement to soft position 2 and 3 may be combined at a later point but for now they're separated for code simplicity)
      else if (m_armEncoder.GetPosition() == v && m_grabberEncoder.GetPosition() > y) {
        m_armMotor.Set(0);
        m_grabberMotor.Set(-0.25);
      }

      //arm rotates down to floor to final position
      else if (m_armEncoder.GetPosition() < z) {
        m_grabberMotor.Set(0);
        m_armMotor.Set(0.25);
      }

      else {
        m_grabberMotor.Set(0); 
        m_armMotor.Set(0);
      } 
}
else {
  m_armMotor.Set(0);
  m_grabberMotor.Set(0);
}
if (armButtonReturn) {
  //arm rotates back to vertical
  if (m_armEncoder.GetPosition() == z && m_grabberEncoder.GetPosition() == y) {
    if (m_armEncoder.GetPosition() > v) {
    m_armMotor.Set(-0.25);
    m_grabberMotor.Set(0);
    }
    //grabber rotates
    else if (m_armEncoder.GetPosition() == v && m_grabberEncoder.GetPosition() < x) {
      m_armMotor.Set(0);
      m_grabberMotor.Set(0.25);
    }
    //arm rotates to initial position
    else if (m_armEncoder.GetPosition() > 0 && m_grabberEncoder.GetPosition() == x) {
      m_armMotor.Set(-0.25);
      m_grabberMotor.Set(0);
    }
    //grabber rotates to initial position
    else if (m_armEncoder.GetPosition() == 0 && m_grabberEncoder.GetPosition() > 0) {
      m_armMotor.Set(0);
      m_grabberMotor.Set(-0.25);
    }
    else {
      m_armMotor.Set(0);
      m_grabberMotor.Set(0);
    }
    
  }
}
else {
  m_armMotor.Set(0);
  m_grabberMotor.Set(0);
}

/*
  if (grabberButton) {                                         // Low Trigger toggles the piston
      if (!lowTriggerToggle);
        m_grabberPistonLeft.Set(frc::DoubleSolenoid::Value::kForward);
        lowTriggerToggle = true;
  }
    else {
        m_grabberPistonLeft.Set(frc::DoubleSolenoid::Value::kReverse);
        lowTriggerToggle = false;
*/  

}



//bifle


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

