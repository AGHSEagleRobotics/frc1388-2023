// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GyroSubsystem extends SubsystemBase {
  /** Creates a new GyroSubsystem. */
  private ADIS16470_IMU m_gyro16470;
  private ADIS16448_IMU m_gyro16448;

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


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
