// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DataLogManager;
public class GyroSubsystem extends SubsystemBase {
  /** Creates a new GyroSubsystem. */
  private ADIS16470_IMU m_gyro16470;
  private ADIS16448_IMU m_gyro16448;
  private MultiChannelADIS m_gyro16470Multi;
  private int counter;

  private DataLog m_log = DataLogManager.getLog();
  private DoubleLogEntry m_logGyroZ = new DoubleLogEntry(m_log, "/robot/GyroZ");
  private DoubleLogEntry m_logGyroX = new DoubleLogEntry(m_log, "/robot/GyroX");
  private DoubleLogEntry m_logGyroY = new DoubleLogEntry(m_log, "/robot/GyroY");
  
  
  private enum gyroType{
    ADIS16448,
    ADIS16470,
    ADIS16470Multi
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
  public GyroSubsystem(MultiChannelADIS gyro){
    m_gyro16470Multi = gyro;
    m_gyro16470Multi.resetAllAngles();
    m_gyroType = gyroType.ADIS16470Multi;
  }
  
// robot orientation is x foward y left and z up
// relative to top side of the roborio(the part with the usb ports)
// on the 16470, y is right, x is backwards, and z is up
// on the 16448, y is right, x is forwards, and z is down
// set from the robot's orientation rather than the gyro's orientation
  public double getXAngle() {
    double xAngle;
    switch (m_gyroType) {
      case ADIS16470:
        xAngle = -m_gyro16470.getYComplementaryAngle(); // Lint
//        xAngle = -m_gyro16470.getXComplementaryAngle(); // RoboRio
        break;

      case ADIS16448:
        xAngle = -m_gyro16448.getGyroAngleY(); // Lint
//        xAngle = m_gyro16448.getGyroAngleX(); // RoboRio
        break;

      case ADIS16470Multi:
        xAngle = -m_gyro16470Multi.getGyroAngleX(); // Lint
//        xAngle = m_gyro16470Multi.getGyroAngleY(); // RoboRio
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
        yAngle = m_gyro16470.getXComplementaryAngle(); // Lint
//        yAngle = -m_gyro16470.getYComplementaryAngle(); // RoboRio
        break;

      case ADIS16448:
        yAngle = -m_gyro16448.getGyroAngleX(); // Lint
//        yAngle = -m_gyro16448.getGyroAngleY(); // RoboRio
        break;
      
      case ADIS16470Multi:
        yAngle = -m_gyro16470Multi.getGyroAngleY(); // Lint
//        yAngle = -m_gyro16470Multi.getGyroAngleX(); // RoboRio
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
        zAngle = -m_gyro16448.getGyroAngleZ();
        break;

      case ADIS16470Multi:
        zAngle = m_gyro16470Multi.getGyroAngleZ(); // Lint
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
    //counter++;
 //   if(counter>=20){
        SmartDashboard.putNumber("Robot Z Angle", getZAngle());
        SmartDashboard.putNumber("Robot X Angle", getXAngle());
        SmartDashboard.putNumber("Robot Y Angle", getYAngle());    
    //  counter = 0;
    //}   
    m_logGyroZ.append(getZAngle());
    m_logGyroX.append(getXAngle());
    m_logGyroY.append(getYAngle());
    
    //competition dashboard
    SmartDashboard.putNumber("Competition/Yaw", getZAngle());
    SmartDashboard.putNumber("Competition/Roll", getXAngle());
    SmartDashboard.putNumber("Competition/Pitch", getYAngle());
  }
}
