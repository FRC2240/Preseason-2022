// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <frc/WPILib.h>
#include "rev/CANSparkMax.h"
#include <frc/drive/DifferentialDrive.h>
#include <frc/smartdashboard/SendableChooser.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;

 private:
  frc::SendableChooser<std::string> m_chooser;
  // Miscellaneous Motor IDs
  // Neos only, Falcons are somewhere else
  const std::int armMotorID = 1;
  const std::int grabberMotorID = 2;
  const std::int leftClimbMotorID = 3;
  const std::int rightClimbMotorID = 4;

  //TODO: DRIVE CODE

  //"Joystick"
  frc::Joystick m_stick{0};
  const std::int leftControlStick = 1; //used to move
  const std::int rightControlStick = 4; //used to rotate

  //Binding motors to controllers, season one, episode four
  //Neo motors
  rev::CANSparkMax m_armMotor{armMotorID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_grabberMotor{grabberMotorID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftClimbMotor{leftClimbMotorID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightClimbMotor{rightClimbMotorID, rev::CANSparkMax::MotorType::kBrushless};

  rev::CANEncoder m_armEncoder = m_armMotor.GetEncoder();
  rev::CANEncoder m_grabberEncoder = m_grabberMotor.GetEncoder();
  rev::CANEncoder m_leftClimbEncoder = m_leftClimbMotor.GetEncoder();
  rev::CANEncoder m_rightClimbEncoder = m_rightClimbMotor.GetEncoder();

  // Limelight
  //Copied from 2021
  std::shared_ptr<NetworkTable> m_table = nt::NetworkTableInstance::GetDefault().GetTable("limelight-scorpio");
  double tx_OFFSET = 0.0; // old = 3.0



};

