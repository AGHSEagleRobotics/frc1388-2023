// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.Key;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;

public class DriveTrain extends SubsystemBase {
  private final WPI_TalonFX m_leftFront;
  private final WPI_TalonFX m_leftBack;
  private final WPI_TalonFX m_rightFront;
  private final WPI_TalonFX m_rightBack;

  private final DifferentialDrive m_differentialDrive;


  /** Creates a new DriveTrain. */
  public DriveTrain(WPI_TalonFX leftFront, WPI_TalonFX leftBack, WPI_TalonFX rightFront, WPI_TalonFX rightBack) {
    m_leftFront = leftFront;
    m_leftBack = leftBack;
    m_rightFront = rightFront;
    m_rightBack = rightBack;

    m_leftBack.follow(m_leftFront);
    m_rightBack.follow(m_rightFront);
    
    // set all motors to brake
    m_leftFront.setNeutralMode(NeutralMode.Brake);
    m_leftBack.setNeutralMode(NeutralMode.Brake);
    m_rightFront.setNeutralMode(NeutralMode.Brake);
    m_rightBack.setNeutralMode(NeutralMode.Brake);
    
    // Invert left motors
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
 * gets left encoder distance
 * @return distance in inches
 */
  public double getLeftEncoderDistance(){
    return m_leftFront.getSelectedSensorPosition()
    * DriveTrainConstants.INCHES_PER_ENCODER_UNITS;
  }

  /**
   * set encoder distance
   * 
   * @param distance in inches
   */
  public void setLeftEncoderDistance(double distance) {
    m_leftFront.setSelectedSensorPosition(distance / DriveTrainConstants.INCHES_PER_ENCODER_UNITS);
    
  }

  /**
   * set encoder distance
   * 
   * @param distance in inches
   */
  public void setRightEncoderDistance(double distance) {
    m_rightFront.setSelectedSensorPosition(distance / DriveTrainConstants.INCHES_PER_ENCODER_UNITS);
    
  }

  /**
   * gets right encoder distance
   * 
   * @return distance in inches
   */
  public double getRightEncoderDistance() {
    return m_rightFront.getSelectedSensorPosition()
        * DriveTrainConstants.INCHES_PER_ENCODER_UNITS;
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("leftDistance",getLeftEncoderDistance());
    SmartDashboard.putNumber("rightDistance",getRightEncoderDistance());
    // This method will be called once per scheduler run
  }
}
