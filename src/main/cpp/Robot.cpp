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

void Robot::ArmDeploy() {
  //Arm functionality coming soon!
}

void Robot::ArmRetract(){
  //Any day now
}

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
  if (autoTimer.Get() <= 0.1){ bool isHolding = true; }

    //limlight network tables?
    //These don't show up in the 2021 bot's code but they are in the docs. Comment out if things don't want to work.

    std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight-wumpus");
    double targetArea = table->GetNumber("ta", 0.0);
    //double targetSkew = table->GetNumber("ts", 0.0);

    double ty = table->GetNumber("ty", 0.0);
    //double tx = table->GetNumber("tx", 0.0);

    if (isHolding == true) {

    if (abs(ty) >= 1) {
        if (ty <= 1) {
            m_robotDrive.DriveCartesian(0.5,0,ty);
        }
        if (ty >= 1){
            m_robotDrive.DriveCartesian(0.5,0,-ty);
        }
    }
  else {
    if (targetArea < 0.25){
      m_robotDrive.DriveCartesian(0.5, 0, 0);
    }
    else if (targetArea > 0.25) { //placeholder for 
        m_grabberPistonLeft.Set(frc::DoubleSolenoid::Value::kReverse);
        m_grabberPistonRight.Set(frc::DoubleSolenoid::Value::kReverse);
      isHolding = false;
    }
      else {
      m_robotDrive.DriveCartesian(0.5, 0, 0);
      }
    }
  }
  else if (isHolding == false) {
    m_robotDrive.DriveCartesian(-0.5, 0, 0);   
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
  //bool armDeployed = false;
  //bool grabberDeployed = false;
  
    //void fighterMode() { 
      // function to read controls for the joystick
        driveY = m_stick.GetRawAxis(0);                    //Possibly Reversed
        driveX = m_stick.GetRawAxis(1);
        driveZ = m_stick.GetRawAxis(5); //Possibly reversed
        //double Throttle = m_stick.GetRawAxis(3); //Change to whatever the Z axis is

//all button bindings need to be tested
        climbButton = m_stick.GetRawButton(2); //Fire!

        armButtonDeploy = m_stick.GetRawButtonPressed(1); //Misnomer. You don't need to hold the button
        armButtonReturn = m_stick.GetRawButtonReleased(2); // don't know
        grabberButton = m_stick.GetRawButtonPressed(11);
        //double buttonA = m_stick.GetRawButtonPressed(3);
        //double buttonB = m_stick.GetRawButtonPressed(3);
        //double buttonC = m_stick.GetRawButtonPressed(4);
        //double buttonE = m_stick.GetRawButtonPressed(8);
        
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

  m_robotDrive.DriveCartesian(-driveY, driveX, -driveZ); //https://docs.wpilib.org/en/stable/docs/software/actuators/wpi-drive-classes.html
  

  if (climbButton /*&& m_climbEncoder.GetVelocity() < 500*/) {
    m_climbMotor.Set(-0.50);
  } 
  else {
    m_climbMotor.Set(0.0);
  }



  //for testing button binding
  if (m_stick.GetRawButton(9)) {
    m_armPIDController.SetReference(m_armRotations[1], rev::ControlType::kSmartMotion);
    //std::cout << "button pressed = " << m_armRotations[1] << "\n";
    //std::cout << "arm encoder position: " << m_armEncoder.GetPosition() << "\n";
  } else {
    m_armPIDController.SetReference(0.0, rev::ControlType::kSmartMotion);
  }

  //if (m_stick.GetRawButtonPressed(9)) {
    if (m_armEncoder.GetPosition() < 160.7) {
      //m_armPIDController.SetReference(m_armRotations[1], rev::ControlType::kSmartMotion);
      double grabberPosition = CalcGrabberPositionOne(m_armEncoder.GetPosition());
      std::cout << "Arm: " << m_armEncoder.GetPosition() << "  wrist: "
            << m_grabberEncoder.GetPosition() << "  calc: " << grabberPosition << "\n";
      m_grabberPIDController.SetReference(grabberPosition, rev::ControlType::kPosition);
    }
    else if (m_armEncoder.GetPosition() >= 160.7 && m_armEncoder.GetPosition() <= 186.2) {
    //m_armPIDController.SetReference(m_armRotations[2], rev::ControlType::kSmartMotion);{
      double grabberPosition = CalcGrabberPositionTwo(m_armEncoder.GetPosition());
      std::cout << "Arm: " << m_armEncoder.GetPosition() << "  wrist: "
            << m_grabberEncoder.GetPosition() << "  calc: " << grabberPosition << "\n";
      m_grabberPIDController.SetReference(grabberPosition, rev::ControlType::kPosition);
    }
  //}


  

  
  

  if (grabberButton) {                                         // Low Trigger toggles the piston
  std::cout << "button pressed \n";
      if (!m_lowTriggerToggle) { 
        m_grabberPistonLeft.Set(frc::DoubleSolenoid::Value::kForward);
        m_grabberPistonRight.Set(frc::DoubleSolenoid::Value::kForward); //not sure about this
        m_lowTriggerToggle = true;
  }
    else {
        m_grabberPistonLeft.Set(frc::DoubleSolenoid::Value::kReverse);
        m_grabberPistonRight.Set(frc::DoubleSolenoid::Value::kReverse);
        m_lowTriggerToggle = false;
  }

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

  std::cout << "Init PID: " << m_grabberCoeff.kP << " " << m_grabberCoeff.kI << " " << m_grabberCoeff.kD
            << " " << m_grabberCoeff.kIz << " " << m_grabberCoeff.kFF << " " << m_grabberCoeff.kMinOutput
            << " " << m_grabberCoeff.kMaxOutput << "\n";


  // default smart motion coefficients
  //double kMaxVel = 550, kMinVel = 0, kMaxAcc = 250, kAllErr = 0;
  //double kMaxVel = 6000, kMinVel = 0, kMaxAcc = 3000, kAllErr = 0;
  //double kMaxVel = 9000, kMinVel = 0, kMaxAcc = 5000, kAllErr = 0;
  double kMaxVel = 4500, kMinVel = 0, kMaxAcc = 2500, kAllErr = 0;


  m_armPIDController.SetSmartMotionMaxVelocity(kMaxVel);
  m_armPIDController.SetSmartMotionMinOutputVelocity(kMinVel);
  m_armPIDController.SetSmartMotionMaxAccel(kMaxAcc);
  m_armPIDController.SetSmartMotionAllowedClosedLoopError(kAllErr);
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
  p   = frc::SmartDashboard::GetNumber("Grabber P Gain", 0);
  i   = frc::SmartDashboard::GetNumber("Grabber I Gain", 0);
  d   = frc::SmartDashboard::GetNumber("Grqbber D Gain", 0);
  min = frc::SmartDashboard::GetNumber("Grabber Min Output", 0);
  max = frc::SmartDashboard::GetNumber("Grabber Max Output", 0);

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

void Robot::TestInit() {
  ReadDashboard(); 
  InitializePIDControllers();
  std::cout << "TestInit\n";
}

void Robot::TestPeriodic() {
        //m_grabberMotor.Set(0.3);
        //m_grabberPIDController.SetReference(100.0, rev::ControlType::kPosition);

        std::cout << "Arm: " << m_armEncoder.GetPosition() << "  Wrist: " << m_grabberEncoder.GetPosition()  << "\n";

}


double Robot::CalcGrabberPositionOne(double arm) {

  return  0.386*arm + 1.26;
}
double Robot::CalcGrabberPositionTwo(double arm) {
  return 1054 + -11.1*arm + 0.0305*arm*arm; 
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif

