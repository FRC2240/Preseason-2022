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

#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

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
    //"Joystick"
  frc::Joystick m_stick{0};

  //TODO: DRIVE CODE
  WPI_TalonFX m_frontRightMotor = {5};
  WPI_TalonFX m_backRightMotor = {6};
  WPI_TalonFX m_frontLeftMotor = {7}; 
  WPI_TalonFX m_backLeftMotor = {8}; 
//attempt initialize mecanum drive
frc::MecanumDrive m_robotDrive{m_frontRightMotor, m_backRightMotor, m_frontLeftMotor, m_backLeftMotor};

//another attempt to initialize mecanum drive 
/*
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
*/


//Binding motors to controllers, season one, episode four
  //Neo motors
  static const int armMotorDeviceID = 1;
  static const int grabberMotorDeviceID = 2; //the "wrist"
  static const int climbMotorDeviceID = 3;
  

  rev::CANSparkMax m_armMotor{armMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_grabberMotor{grabberMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_climbMotor{climbMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};

  rev::CANEncoder m_armEncoder = m_armMotor.GetEncoder();
  rev::CANEncoder m_grabberEncoder = m_grabberMotor.GetEncoder();
  rev::CANEncoder m_climbEncoder = m_climbMotor.GetEncoder();

  //timer
  frc::Timer autoTimer;

  // Limelight
  //Copied from 2021
  //something is throwing an error here and idk what std::shared_ptr<NetworkTable> m_table = nt::NetworkTableInstance::GetDefault().GetTable("limelight-scorpio"); 
  double tx_OFFSET = 0.0; // old = 3.0


  //pneumatics//
  frc::DoubleSolenoid m_grabberPistonLeft{0, 7};
  frc::DoubleSolenoid m_grabberPistonRight {1,6};

};

