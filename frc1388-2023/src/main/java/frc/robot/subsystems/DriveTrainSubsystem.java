// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;

import frc.robot.Constants.DriveTrainConstants;

public class DriveTrainSubsystem extends SubsystemBase {
  private final WPI_TalonFX m_leftFront;
  private final WPI_TalonFX m_leftBack;
  private final WPI_TalonFX m_rightFront;
  private final WPI_TalonFX m_rightBack;

  private final DifferentialDrive m_differentialDrive;


  /** Creates a new DriveTrain. */
  public DriveTrainSubsystem(WPI_TalonFX leftFront, WPI_TalonFX leftBack, WPI_TalonFX rightFront, WPI_TalonFX rightBack) {
    m_leftFront = leftFront;
    m_leftBack = leftBack;
    m_rightFront = rightFront;
    m_rightBack = rightBack;
   
    m_leftFront.configFactoryDefault();
    m_leftBack.configFactoryDefault();
    m_rightFront.configFactoryDefault();
    m_rightBack.configFactoryDefault();

    m_leftBack.follow(m_leftFront);
    m_rightBack.follow(m_rightFront);
    
    // set all motors to brake
    m_leftFront.setNeutralMode(NeutralMode.Brake);
    m_leftBack.setNeutralMode(NeutralMode.Brake);
    m_rightFront.setNeutralMode(NeutralMode.Brake);
    m_rightBack.setNeutralMode(NeutralMode.Brake);
    
    // Invert left motors
    // Controller Port
    m_leftFront.setInverted(true);
    m_leftBack.setInverted(true);

    // Invert right motors
    m_rightFront.setInverted(false);
    m_rightBack.setInverted(false);

    // set distance to zero
    setLeftEncoderDistance(0);
    setRightEncoderDistance(0);
    // Differential Drive
    m_differentialDrive = new DifferentialDrive(m_leftFront, m_rightFront);
    m_differentialDrive.setDeadband(0);

    // constant speed motor setting and PID
    m_leftFront.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    //m_leftFront.setSensorPhase(false);
    m_leftFront.configClosedloopRamp(DriveTrainConstants.CLOSED_LOOP_RAMP_RATE);
    m_leftFront.config_kF(DriveTrainConstants.PID_IDX, DriveTrainConstants.GAINS_VELOCITY_F);
    m_leftFront.config_kP(DriveTrainConstants.PID_IDX, DriveTrainConstants.GAINS_VELOCITY_P);
    m_leftFront.config_kI(DriveTrainConstants.PID_IDX, DriveTrainConstants.GAINS_VELOCITY_I);
    m_leftFront.config_kD(DriveTrainConstants.PID_IDX, DriveTrainConstants.GAINS_VELOCITY_D);

    m_rightFront.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    //m_leftFront.setSensorPhase(false);
    m_rightFront.configClosedloopRamp(DriveTrainConstants.CLOSED_LOOP_RAMP_RATE);
    m_rightFront.config_kF(DriveTrainConstants.PID_IDX, DriveTrainConstants.GAINS_VELOCITY_F);
    m_rightFront.config_kP(DriveTrainConstants.PID_IDX, DriveTrainConstants.GAINS_VELOCITY_P);
    m_rightFront.config_kI(DriveTrainConstants.PID_IDX, DriveTrainConstants.GAINS_VELOCITY_I);
    m_rightFront.config_kD(DriveTrainConstants.PID_IDX, DriveTrainConstants.GAINS_VELOCITY_D);
  }
    

  public void arcadeDrive( double xSpeed, double zRotation) {
    m_differentialDrive.arcadeDrive(xSpeed, zRotation);
  }
  public void curvatureDrive( double xSpeed, double zRotation, boolean allowTurnInPlace) {
    m_differentialDrive.curvatureDrive(xSpeed, zRotation, allowTurnInPlace);
  }
  public void tankDrive( double leftSpeed, double rightSpeed) {
    m_differentialDrive.tankDrive(leftSpeed, rightSpeed);
  }
  
  /**
   * Constant speed drive method for differential drive platform.
   *
   * <p> This drives the robot motors at a constant speed rather than a constant voltage
   *
   * @param speed The robot's speed along the X axis in inches per second. Forward is positive.
   */
  public void constantSpeedDrive(double speed) {

    //Velocity is in ticks per 100 miliseconds
    double velocity = speed / DriveTrainConstants.INCHES_PER_ENCODER_UNITS / DriveTrainConstants.SENSOR_CYCLES_PER_SECOND;
    m_leftFront.set(ControlMode.Velocity, velocity);      
    m_rightFront.set(ControlMode.Velocity, velocity);

    SmartDashboard.putNumber("velocity", velocity);
    SmartDashboard.putNumber("robot velocity", m_leftFront.getSelectedSensorVelocity());
    // SmartDashboard.putNumber("integrator", m_leftFront.getIntegralAccumulator());
    // SmartDashboard.putNumber("derivative", m_leftFront.getErrorDerivative());
    // SmartDashboard.putNumber("error", m_leftFront.getClosedLoopError());
  }

  public void setNeutralMode(NeutralMode mode) {
    m_leftFront.setNeutralMode(mode);
    m_leftBack.setNeutralMode(mode);
    m_rightFront.setNeutralMode(mode);
    m_rightBack.setNeutralMode(mode);
  }

  public void resetLeftEncoder() {
    m_leftFront.setSelectedSensorPosition(0);
  }

  // gets left encoder distance in inches
  public double getLeftEncoderDistance() {
    return m_leftFront.getSelectedSensorPosition()
        * DriveTrainConstants.INCHES_PER_ENCODER_UNITS;
  }

  // sets encoder distance in inches
  public void setLeftEncoderDistance(double distance) {
    m_leftFront.setSelectedSensorPosition(distance / DriveTrainConstants.INCHES_PER_ENCODER_UNITS);
  }

  public void resetRightEncoder() {
    m_rightFront.setSelectedSensorPosition(0);
  }

  // sets encoder distance in inches
  public void setRightEncoderDistance(double distance) {
    m_rightFront.setSelectedSensorPosition(distance / DriveTrainConstants.INCHES_PER_ENCODER_UNITS);
  }

  // gets right encoder distance in inches
  public double getRightEncoderDistance() {
    return m_rightFront.getSelectedSensorPosition()
        * DriveTrainConstants.INCHES_PER_ENCODER_UNITS;
  }

  public void resetEncoders() {
    resetRightEncoder();
    resetLeftEncoder();
  }
  
  public void setDeadbandZero() {
    m_differentialDrive.setDeadband(0); 
  }
  
  
  /**feet/sec */
  public double getRobotSpeed() {
    double encoderSpeed = (m_leftFront.getSelectedSensorVelocity() + m_rightFront.getSelectedSensorVelocity()) / 2;
    return encoderSpeed * DriveTrainConstants.INCHES_PER_ENCODER_UNITS * 10 / 12;
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //debug===============
    SmartDashboard.putNumber("robot speed in feet per sec", getRobotSpeed());
    //debug=end============
  }


}
