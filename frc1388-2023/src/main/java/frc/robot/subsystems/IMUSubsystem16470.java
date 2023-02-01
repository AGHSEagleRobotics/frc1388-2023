// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class IMUSubsystem16470 extends SubsystemBase {
  /** Creates a new GyroSubsystem. */
  private final ADIS16470_IMU m_gyro;
  private int counter;
  private static final Logger log = LogManager.getLogger(IMUSubsystem16470.class);

  public IMUSubsystem16470(ADIS16470_IMU gyro) {
    m_gyro = gyro;
    m_gyro.setYawAxis(IMUAxis.kZ);
    m_gyro.calibrate();
    m_gyro.reset();
   
  }

  // getters for gyro angles and reset
  public double getXGyro() {
    return m_gyro.getXComplementaryAngle();
  }
  public double getYGyro() {
    return m_gyro.getYComplementaryAngle();
  }
  public double getZGyro() {
    return m_gyro.getAngle();
  }
  public void resetGyro() {
    m_gyro.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    counter++;
    if(counter == 20){
      System.out.println("Test 1: Rotation: " + m_gyro.getAngle() + ", X angle: " + 
      m_gyro.getXComplementaryAngle() + ", Y angle: " + m_gyro.getYComplementaryAngle());
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