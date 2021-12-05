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
  
 InitializeDashboard();
  InitializePIDControllers();
  m_armEncoder.SetPosition(0);
  m_grabberEncoder.SetPosition(0);
  m_climbEncoder.SetPosition(0);

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
    //limlight network tables?
    //These don't show up in the 2021 bot's code but they are in the docs. Comment out if things don't want to work.

    std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    double targetArea = table->GetNumber("ta", 0.0);
    double targetSkew = table->GetNumber("ts", 0.0);

    double ty = m_table->GetNumber("ty", 0.0);
    double tx = m_table->GetNumber("tx", 0.0);

    if (abs(ty) >= 1) {
        if (ty <= 1) {
            m_robotDrive.DriveCartesian(0.5,0,ty);
        }
        if (ty >= 1){
            m_robotDrive.DriveCartesian(0.5,0,-ty);
        }
    }
}
/*
  autoTimer.Start();
  if (autoTimer.Get() <= 0.25) {
    m_robotDrive.DriveCartesian(0.5,0,0);
  } else {
    // Default Auto goes here
  }
}
 */
void Robot::TeleopInit() {
  ReadDashboard(); 
  InitializePIDControllers();
  std::cout << "arm rotations 1: " << m_armRotations[1] << "\n"; 
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
  bool armDeployed = false;
  bool grabberDeployed = false;
  
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
        grabberButton = m_stick.GetRawButtonPressed(6);
        double buttonA = m_stick.GetRawButtonPressed(3);
        double buttonB = m_stick.GetRawButtonPressed(3);
        double buttonC = m_stick.GetRawButtonPressed(4);
        double buttonE = m_stick.GetRawButtonPressed(8);
        
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
  bool lowTriggerToggle;
  bool triggerToggle;

  m_robotDrive.DriveCartesian(-driveY, driveX, -driveZ); //https://docs.wpilib.org/en/stable/docs/software/actuators/wpi-drive-classes.html
  

  if (climbButton && m_climbEncoder.GetVelocity() < 500) {
    m_climbMotor.Set(0.25); //don't know if this will work
  }

//for testing button binding

  if (m_stick.GetRawButton(m_button)) {
    m_armPIDController.SetReference(m_armRotations[1], rev::ControlType::kPosition);
    std::cout << "button pressed = " << m_armRotations[1] << "\n";
    std::cout << "arm encoder position: " << m_armEncoder.GetPosition() << "\n";
  }

  
  
/*
  if (armButtonDeploy) {
    //This raises/lowers the arm for putting on the gear
    m_armPIDController.SetReference(m_armRotations[1], rev::ControlType::kPosition);
    m_grabberPIDController.SetReference(m_grabberRotations[0], rev::ControlType::kPosition); //figure out equation
    armDeployed = true;
    if (armDeployed) {
      m_grabberPIDController.SetReference(m_grabberRotations[1], rev::ControlType::kPosition);
      grabberDeployed = true;
    }
    

/*
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
else {
  m_armMotor.Set(0);
  m_grabberMotor.Set(0);
}


if (armButtonReturn) {
    m_armPIDController.SetReference(m_armRotations[0], rev::ControlType::kPosition);
    m_grabberPIDController.SetReference(m_grabberRotations[0], rev::ControlType::kPosition); //grabber should automatically go back up by the equation right??
    armDeployed = false;
    grabberDeployed = false;
  
/*  
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

}
else {
  m_armMotor.Set(0);
  m_grabberMotor.Set(0);
}

*/

  if (grabberButton) {                                         // Low Trigger toggles the piston
      if (!lowTriggerToggle);
        m_grabberPistonLeft.Set(frc::DoubleSolenoid::Value::kForward);
        m_grabberPistonRight.Set(frc::DoubleSolenoid::Value::kForward); //not sure about this
        lowTriggerToggle = true;
  }
    else {
        m_grabberPistonLeft.Set(frc::DoubleSolenoid::Value::kReverse);
        m_grabberPistonRight.Set(frc::DoubleSolenoid::Value::kReverse);
        lowTriggerToggle = false;
  }

}


//bifle


// https://cynosure.neocities.org/topsneaky.html



void Robot::InitializePIDControllers() { 
  m_armPIDController.SetP(m_armCoeff.kP);
  m_armPIDController.SetI(m_armCoeff.kI);
  m_armPIDController.SetD(m_armCoeff.kD);
  m_armPIDController.SetIZone(m_armCoeff.kIz);
  m_armPIDController.SetFF(m_armCoeff.kFF);
  m_armPIDController.SetOutputRange(m_armCoeff.kMinOutput, m_armCoeff.kMaxOutput);

  m_grabberPIDController.SetP(m_grabberCoeff.kP);
  m_grabberPIDController.SetI(m_grabberCoeff.kI);
  m_grabberPIDController.SetD(m_grabberCoeff.kD);
  m_grabberPIDController.SetIZone(m_grabberCoeff.kIz);
  m_grabberPIDController.SetFF(m_grabberCoeff.kFF);
  m_grabberPIDController.SetOutputRange(m_grabberCoeff.kMinOutput, m_grabberCoeff.kMaxOutput);
}

void Robot::InitializeDashboard() {
  // display PID coefficients on SmartDashboard
  frc::SmartDashboard::PutNumber("Arm P Gain",              m_armCoeff.kP);
  frc::SmartDashboard::PutNumber("Arm I Gain",              m_armCoeff.kI);
  frc::SmartDashboard::PutNumber("Arm D Gain",              m_armCoeff.kD);
  frc::SmartDashboard::PutNumber("Arm Max Output",          m_armCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Arm Min Output",          m_armCoeff.kMinOutput);

  frc::SmartDashboard::PutNumber("Grabber P Gain",            m_grabberCoeff.kP);
  frc::SmartDashboard::PutNumber("Grabber I Gain",            m_grabberCoeff.kI);
  frc::SmartDashboard::PutNumber("Grabber D Gain",            m_grabberCoeff.kD);
  frc::SmartDashboard::PutNumber("Grabber Max Output",        m_grabberCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Grabber Min Output",        m_grabberCoeff.kMinOutput);

  frc::SmartDashboard::PutNumber("Arm Rotations Level 0",   m_armRotations[0]);
  frc::SmartDashboard::PutNumber("Arm Rotations Level 1",   m_armRotations[1]);

  frc::SmartDashboard::PutNumber("Grabber Rotations Level 0", m_grabberRotations[0]);
  frc::SmartDashboard::PutNumber("Grabber Rotations Level 1", m_grabberRotations[1]);

  frc::SmartDashboard::PutNumber("Button number: ", m_button);
}

void Robot::ReadDashboard () {
  double p, i, d, min, max;

  // read PID coefficients from SmartDashboard
  p   = frc::SmartDashboard::GetNumber("Arm P Gain", 0);
  std::cout << "Read Dashboard arm p gain: " << p << "\n";
  i   = frc::SmartDashboard::GetNumber("Arm I Gain", 0);
  std::cout << "Read Dashboard arm i gain: " << p << "\n";
  d   = frc::SmartDashboard::GetNumber("Arm D Gain", 0);
  std::cout << "Read Dashboard arm d gain: " << p << "\n";
  min = frc::SmartDashboard::GetNumber("Arm Min Output", 0);
  max = frc::SmartDashboard::GetNumber("Arm Max Output", 0);

  // If PID coefficients on SmartDashboard have changed, write new values to controller
  if ((p != m_armCoeff.kP)) { m_armPIDController.SetP(p); m_armCoeff.kP = p; }
  if ((i != m_armCoeff.kI)) { m_armPIDController.SetI(i); m_armCoeff.kI = i; }
  if ((d != m_armCoeff.kD)) { m_armPIDController.SetD(d); m_armCoeff.kD = d; }
  if ((max != m_armCoeff.kMaxOutput) || (min != m_armCoeff.kMinOutput)) { 
    m_armPIDController.SetOutputRange(min, max); 
    m_armCoeff.kMinOutput = min; m_armCoeff.kMaxOutput = max; 
  }
  p   = frc::SmartDashboard::GetNumber("Gear P Gain", 0);
  i   = frc::SmartDashboard::GetNumber("Gear I Gain", 0);
  d   = frc::SmartDashboard::GetNumber("Gear D Gain", 0);
  min = frc::SmartDashboard::GetNumber("Gear Min Output", 0);
  max = frc::SmartDashboard::GetNumber("Gear Max Output", 0);

  // If PID coefficients on SmartDashboard have changed, write new values to controller
  if ((p != m_grabberCoeff.kP)) { m_grabberPIDController.SetP(p); m_grabberCoeff.kP = p; }
  if ((i != m_grabberCoeff.kI)) { m_grabberPIDController.SetI(i); m_grabberCoeff.kI = i; }
  if ((d != m_grabberCoeff.kD)) { m_grabberPIDController.SetD(d); m_grabberCoeff.kD = d; }
  if ((max != m_grabberCoeff.kMaxOutput) || (min != m_grabberCoeff.kMinOutput)) { 
    m_grabberPIDController.SetOutputRange(min, max); 
    m_grabberCoeff.kMinOutput = min; m_grabberCoeff.kMaxOutput = max; 
}
  m_armRotations[0] = frc::SmartDashboard::GetNumber("Arm Rotations Level 0",   m_armRotations[0]);
  m_armRotations[1] = frc::SmartDashboard::GetNumber("Arm Rotations Level 1",   m_armRotations[1]);
  std::cout << "Read Dashboard: " << m_armRotations[1] << "\n";

m_button = frc::SmartDashboard::GetNumber("Button number: ", m_button);
std::cout << "Read dashboard button: " << m_button << "\n";

  m_grabberRotations[0] = frc::SmartDashboard::GetNumber("Grabber Rotations Level 0", m_grabberRotations[0]);
  frc::SmartDashboard::GetNumber("Grabber Rotations Level 1", m_grabberRotations[1]);

  }
void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {

}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif

