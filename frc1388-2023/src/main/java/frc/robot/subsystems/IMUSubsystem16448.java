// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class GyroSubsystem extends SubsystemBase {
  /** Creates a new GyroSubsystem. */
  private final ADIS16448_IMU m_gyro;
  private int counter;
  private static final Logger log = LogManager.getLogger(GyroSubsystem.class);

  public GyroSubsystem(ADIS16448_IMU gyro) {
    m_gyro = gyro;
    m_gyro.calibrate();
    m_gyro.reset();
   

  }

  // getters for gyro angles and reset
  public double getXGyro() {
    return m_gyro.getGyroAngleX();
  }
  public double getYGyro() {
    return m_gyro.getGyroAngleY();
  }
  public double getZGyro() {
    return m_gyro.getGyroAngleZ();
  }
  public void resetGyro() {
    m_gyro.reset();
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    counter++;
    if( counter == 20){
      System.out.println("Test 1: Rotation: " + m_gyro.getGyroAngleZ() + 
      ", X angle: " + m_gyro.getGyroAngleX() + ", Y angle: " + m_gyro.getGyroAngleY());
      System.out.println();
      counter = 0;
    }

    
    // log.debug("Y angle: {}", m_gyro.getAngle());

  //    m_gyro.setYawAxis(IMUAxis.kX);
  //    double x = m_gyro.getAngle();
  //    m_gyro.setYawAxis(IMUAxis.kY);
  //    double y = m_gyro.getAngle();
  //    m_gyro.setYawAxis(IMUAxis.kZ);
  //    double z = m_gyro.getAngle();

  //    log.info("Test 2: Rotation: {}, X angle: {}, Y angle: {}", z, x, y);
   }
}
