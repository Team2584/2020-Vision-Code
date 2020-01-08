/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include "ctre/Phoenix.h"
#include "rev/CANSparkMax.h"
#include <iostream>
#include <frc/encoder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>
#include <frc/DigitalSource.h>
#include "frc/WPILib.h"
#include <stdio.h>
#include <memory>
#include <chrono>
#include <thread>
#include <stdlib.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "robotIO.h"
#include "frc/Watchdog.h"
#include <string>
#include "math.h"

using namespace frc;
using namespace std;


//Turn Table Setup
static const int turnTableLeftID = 1, turnTableRightID = 2;
rev::CANSparkMax m_turnTableLeft{turnTableLeftID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax m_turnTableRight{turnTableRightID, rev::CANSparkMax::MotorType::kBrushless};

//Flywheel Setup(Make sure to invert one motor)
static const int flywheelRightID = 3, flywheelLeftID = 4;
rev::CANSparkMax m_flywheelLeft{flywheelLeftID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax m_flywheelRight{flywheelRightID, rev::CANSparkMax::MotorType::kBrushless};

//INIT INPUTS CLASS
robotIO* inputs = new robotIO;

//Create Flywheel PID
rev::CANPIDController m_pidFlywheel = m_flywheelLeft.GetPIDController();
//Create Flywheel Encoder
rev::CANEncoder m_flywheelEncoder = m_flywheelLeft.GetEncoder();

//Turn Table PID coefficients(_T = Turntable coefficient)
double kP_T = 0.65, kI_T = 0.00005, kD_T = 0.05, kIz_T = 0, kFF_T = 0, kMaxOutput_T = 0.3, kMinOutput_T = -0.80;

//Flywheel PID coefficients(_F = Flywheel coefficient)
double kP_F = 6e-5, kI_F = 1e-6, kD_F = 0, kIz_F = 0, kFF_F = 0.000015, kMaxOutput_F = 1.0, kMinOutput_F = -1.0;

//Flywheel motor max RPM
const double MaxRPM = 5700;

void Robot::RobotInit() {

  static const int indexerID = 1;
  Indexer = new WPI_TalonSRX(indexerID);

  // set PID coefficients
    m_pidFlywheel.SetP(kP_F);
    m_pidFlywheel.SetI(kI_F);
    m_pidFlywheel.SetD(kD_F);
    m_pidFlywheel.SetIZone(kIz_F);
    m_pidFlywheel.SetFF(kFF_F);
    m_pidFlywheel.SetOutputRange(kMinOutput_F, kMaxOutput_F);

  //Set Follower Motors
    m_flywheelRight.Follow(m_flywheelLeft);//Make sure one is reversed
    m_turnTableRight.Follow(m_turnTableLeft);

  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
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
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString(
  //     "Auto Selector", kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {
}

  double kP_T = 0.5;//Turn table P constant(random number chosen)
  double kS_F = 580;//Flywheel speed multiplier
  double turretSpeed;
  bool fireReady = false;
  double setSpeed;
  double indexerSpeed = -0.1;

void Robot::TeleopPeriodic() {

  std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  double targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0); //Get horizontal offset from target
  double targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);   //Get vertical offset from target
  double targetArea = table->GetNumber("ta",0.0);                   //Get area of target on screen
  double targetSkew = table->GetNumber("ts",0.0);                   //Get skew of target
  double targetAcquired = table->GetNumber("tv", 0.0);

  //Turret Aiming Function
  if(inputs->getButtonCircle_P() && targetAcquired == true){
    turretSpeed = kP_T * abs(targetOffsetAngle_Horizontal);
  }
  else{
    turretSpeed = inputs->getRStickX_P(); //Turret Manual Control
  }
  m_turnTableLeft.Set(turretSpeed); //Set turret speed

  //Checks for target alignment
  if(abs(targetOffsetAngle_Horizontal) < 3){ //some margin of error
    fireReady = true; //sets ready variable to true
  }
  else{
    fireReady = false;
  }

  if(fireReady = true){
    setSpeed = (1/(targetArea + 0.1)) * kS_F; //speed up flywheer
  }
  else if(fireReady = true && inputs->getShoulderRightHeld_P()){
    indexerSpeed = 0.5; //enable indexer to shoot fuel
  }
  else{
    setSpeed = 300; //idling speed(may remove)
    indexerSpeed = -0.1; //idling speed to keep fuel cells from getting into flywheel
  }
  m_pidFlywheel.SetReference(setSpeed, rev::ControlType::kVelocity); //pid controlled velocity
  Indexer->Set(ControlMode::PercentOutput, indexerSpeed); //indexer speed
   

  //Basic PID velocity example
  /*if(inputs->getButtonCircle()){
    setSpeed = 2500;
  }
  else{
    setSpeed = 0;
  }
  m_pidFlywheel.SetReference(setSpeed, rev::ControlType::kVelocity);*/


}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
