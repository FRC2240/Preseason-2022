// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
//added from 2021 and Phoenix for falcons
#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <frc/controller/RamseteController.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#include "rev/CANSparkMax.h"
#include "ctre/Phoenix.h"


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
  static const int armMotorDeviceID = 1;
  static const int grabberMotorDeviceID = 2;
  static const int leftClimbMotorDeviceID = 3;
  static const int rightClimbMotorDeviceID = 4;

  //"DONE": Drive Code
  //These are two ways I've seen talons initialized, the first set being an augmentation of an initialization of TalonSRX, the second set from the CrossTheRoadELec repository on Github for TalonFX differential drive (both example sets were written in cpp)
  TalonFX frontRightMotor = {5};
  TalonFX frontLeftMotor = {6}; 
  TalonFX backRightMotor = {7};
  TalonFX backLeftMotor = {8}; 

/*
  WPI_TalonFX * _rghtFront = new WPI_TalonFX(5);
	WPI_TalonFX * _rghtFollower = new WPI_TalonFX(7);
	WPI_TalonFX * _leftFront = new WPI_TalonFX(6);
	WPI_TalonFX * _leftFollower = new WPI_TalonFX(8);
*/ 

  //"Joystick"
  frc::Joystick m_stick{0};
  static const int leftControlStick = 1; //used to move
  static const int rightControlStick = 4; //used to rotate

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
  std::shared_ptr<NetworkTable> m_table = nt::NetworkTableInstance::GetDefault().GetTable("limelight-scorpio"); //missing a library?
  double tx_OFFSET = 0.0; // old = 3.0

  static constexpr int kFrontLeftChannel = 0;
  static constexpr int kRearLeftChannel = 1;
  static constexpr int kFrontRightChannel = 2;
  static constexpr int kRearRightChannel = 3;

  static constexpr int kJoystickChannel = 0;

  frc::PWMVictorSPX m_frontLeft{kFrontLeftChannel};
  frc::PWMVictorSPX m_rearLeft{kRearLeftChannel};
  frc::PWMVictorSPX m_frontRight{kFrontRightChannel};
  frc::PWMVictorSPX m_rearRight{kRearRightChannel};
  frc::MecanumDrive m_robotDrive{m_frontLeft, m_rearLeft, m_frontRight,
                                 m_rearRight};



};

