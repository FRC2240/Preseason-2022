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
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

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
  void InitializeDashboard();
  void InitializePIDControllers();
  void ReadDashboard();

 private:

   double CalcGrabberPositionOne(double arm);
   double CalcGrabberPositionTwo(double arm);



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
  frc::MecanumDrive m_robotDrive{m_frontLeft, m_r
*/


//Binding motors to controllers, season one, episode four
  //Neo motors
  static const int armMotorDeviceID = 3;
  static const int grabberMotorDeviceID = 11; //the "wrist"
  static const int climbMotorDeviceID = 1;
  

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

  //PID initialization
  rev::CANPIDController m_armPIDController = m_armMotor.GetPIDController();
  rev::CANPIDController m_grabberPIDController = m_grabberMotor.GetPIDController();

  struct pidCoeff {
    double kP;
    double kI;
    double kD;
    double kIz;
    double kFF;
    double kMinOutput;
    double kMaxOutput;
  };

  // DETERMINE THESE EXPERIMENTALLY!!!!!!!
  pidCoeff m_armCoeff {2.0e-4, 1e-6, 0.0, 0.0, 0.0, -1.0, 1.0};
  pidCoeff m_grabberCoeff {0.13, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0};
  
  double m_armRotations[2] {0.0, 186.2}; //0 is undeployed, 1 is deployed
  double m_grabberRotations[2] {0.0, 0.0}; //0 is undeployed or in the upright position, 1 is deployed

  double m_button; 
  bool m_lowTriggerToggle;

};


