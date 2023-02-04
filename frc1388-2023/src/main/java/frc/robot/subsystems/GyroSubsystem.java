// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import org.ejml.sparse.csc.mult.MatrixVectorMultWithSemiRing_DSCC;

import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GyroSubsystem extends SubsystemBase {
  /** Creates a new GyroSubsystem. */
  private ADIS16470_IMU m_gyro16470;
  private ADIS16448_IMU m_gyro16448;
  private int counter;
  private enum gyroType{
    ADIS16448,
    ADIS16470
  }

  private gyroType m_gyroType;
  
  public GyroSubsystem(ADIS16448_IMU gyro){
    m_gyro16448 = gyro;
    m_gyro16448.reset();
    m_gyroType = gyroType.ADIS16448;
  } 
  public GyroSubsystem(ADIS16470_IMU gyro){
    m_gyro16470 = gyro;
    m_gyro16470.setYawAxis(IMUAxis.kZ);
    m_gyro16470.reset();
    m_gyroType = gyroType.ADIS16470;
  }

  public double getXAngle() {
    double xAngle;
    switch (m_gyroType) {
      case ADIS16470:
        xAngle = m_gyro16470.getXComplementaryAngle();
        break;

      case ADIS16448:
        xAngle = m_gyro16448.getGyroAngleX();
        break;

      default:
        xAngle = 0.0;
        break;
    }
    return xAngle;
  }
    
  public double getYAngle() {
    double yAngle;
    switch (m_gyroType) {
      case ADIS16470:
        yAngle = m_gyro16470.getYComplementaryAngle();
        break;

      case ADIS16448:
        yAngle = m_gyro16448.getGyroAngleY();
        break;

      default:
        yAngle = 0.0;
        break;
    }
    return yAngle;
  }
  
  public double getZAngle() {
    double zAngle;
    switch (m_gyroType) {
      case ADIS16470:
        zAngle = m_gyro16470.getAngle();
        break;

      case ADIS16448:
        zAngle = m_gyro16448.getGyroAngleZ();
        break;

      default:
        zAngle = 0.0;
        break;
    }
    return zAngle;
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    counter++;
    if(counter>=20){
      switch(m_gyroType){
        case ADIS16470:
        SmartDashboard.putNumber("ADIS16470/Z Angle", m_gyro16470.getAngle());
        SmartDashboard.putNumber("ADIS16470/X Angle", m_gyro16470.getXComplementaryAngle());
        SmartDashboard.putNumber("ADIS16470/Y Angle", m_gyro16470.getYComplementaryAngle());    
        break;
        
        case ADIS16448:
        SmartDashboard.putNumber("ADIS16448/Z Angle", m_gyro16448.getGyroAngleZ());
        SmartDashboard.putNumber("ADIS16448/X Angle", m_gyro16448.getGyroAngleX());
        SmartDashboard.putNumber("ADIS16448/Y Angle", m_gyro16448.getGyroAngleY());
        break;
      }
      counter = 0;
    }   
  }
}
