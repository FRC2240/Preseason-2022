// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
#include <frc/drive/MecanumDrive.h>

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

  //TODO: DRIVE CODE
  //These are two ways I've seen talons initialized, the first set being an augmentation of an initialization of TalonSRX, the second set from the CrossTheRoadELec repository on Github for TalonFX differential drive (both example sets were written in cpp)
  
  WPI_TalonFX m_frontRightMotor = {5};
  WPI_TalonFX m_frontLeftMotor = {7}; 
  WPI_TalonFX m_backRightMotor = {6};
  WPI_TalonFX m_backLeftMotor = {8}; 
//attempt initialize mecanum drive
frc::MecanumDrive m_robotDrive{m_frontRightMotor, m_backRightMotor, m_frontLeftMotor, m_backLeftMotor};

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
 // std::shared_ptr<NetworkTable> m_table = nt::NetworkTableInstance::GetDefault().GetTable("limelight-scorpio"); //no idea
  double tx_OFFSET = 0.0; // old = 3.0

  //timer
  frc::Timer autoTimer;

  //pneumatics//
  frc::DoubleSolenoid m_intakeleft{0, 7};

};

