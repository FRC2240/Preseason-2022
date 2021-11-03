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
  const std::int armMotorDeviceID = 1;
  const std::int grabberMotorDeviceID = 2;
  const std::int leftClimbMotorDeviceID = 3;
  const std::int rightClimbMotorDeviceID = 4;

  //TODO: DRIVE CODE

  //"Joystick"
  frc::Joystick m_stick{0};
  const std::int leftControlStickID = 1; //used to move
  const std::int rightControlStickDeviceID = 4; //used to rotate

  //Binding motors to controllers, season one, episode four
  //Neo motors
  rev::CANSparkMax m_armMotor{armMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_grabberMotor{grabberMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftClimbMotor{leftClimbMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightClimbMotor{rightClimbMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};

  rev::CANEncoder m_armEncoder = m_armMotor.GetEncoder();
  rev::CANEncoder m_grabberEncoder = m_grabberMotor.GetEncoder();
  rev::CANEncoder m_leftClimbEncoder = m_leftClimbMotor.GetEncoder();
  rev::CANEncoder m_rightClimbEncoder = m_rightClimbMotor.GetEncoder();

  // Limelight
  //Copied from 2021
  std::shared_ptr<NetworkTable> m_table = nt::NetworkTableInstance::GetDefault().GetTable("limelight-scorpio");
  double tx_OFFSET = 0.0; // old = 3.0



};

