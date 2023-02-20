// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * This class supports logging of miscellaneous data that's not associated with another specific command or subsystem.
 */
public class LoggingSubsystem extends SubsystemBase {

  // Types
  private enum RobotMode {
    DISABLED(0),
    AUTO(1),
    TELEOP(2),
    TEST(3);

    private int value;
    private RobotMode(int value) {
      this.value = value;
    }
  }

  // Fields
  private RobotMode m_robotMode = RobotMode.DISABLED;

  // Data objects
  private BuiltInAccelerometer m_accel = new BuiltInAccelerometer();

  // Log entries
  private DataLog m_log = DataLogManager.getLog();

  private DoubleLogEntry m_logGX;
  private DoubleLogEntry m_logGY;
  private DoubleLogEntry m_logGZ;
  private DoubleLogEntry m_logGTotal;

  private DoubleLogEntry m_logMode;

  /** Creates a new LoggingSubsystem. */
  public LoggingSubsystem() {
    
    if (Constants.Logging.LOG_ACCELERATION)
    {
      m_logGX = new DoubleLogEntry(m_log, "/robot/gX");
      m_logGY = new DoubleLogEntry(m_log, "/robot/gY");
      m_logGZ = new DoubleLogEntry(m_log, "/robot/gZ");
      m_logGTotal = new DoubleLogEntry(m_log, "/robot/gTotal");
    }
  
    m_logMode = new DoubleLogEntry(m_log, "/robot/Mode");

    // Log RoboRIO file system usage
    File logPath = new File("home/lvuser");
    long totalSpace = logPath.getTotalSpace();
    long freeSpace = logPath.getUsableSpace();
    long fileUsage = ((totalSpace - freeSpace) * 100 / totalSpace);
    DataLogManager.log("RoboRIO disk usage: " + fileUsage + "%");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Log accelerations
    if (Constants.Logging.LOG_ACCELERATION) {
      double gX = m_accel.getX();
      double gY = m_accel.getY();
      double gZ = m_accel.getZ();
      double gTotal = Math.sqrt((gX*gX) + (gY*gY) + (gZ*gZ));   // Total acceleration vector magnitude
  
      m_logGX.append(gX);
      m_logGY.append(gY);
      m_logGZ.append(gZ);
      m_logGTotal.append(gTotal);
    }

    // Log robot mode
    RobotMode newMode = m_robotMode;
    if (DriverStation.isDisabled()) {
      newMode = RobotMode.DISABLED;
    } else if (DriverStation.isAutonomous()) {
      newMode = RobotMode.AUTO;
    } else if (DriverStation.isTeleop()) {
      newMode = RobotMode.TELEOP;
    } else if (DriverStation.isTest()) {
      newMode = RobotMode.TEST;
    }
    if (newMode != m_robotMode) {
      m_logMode.append(newMode.value);
    }
    m_robotMode = newMode;
    
  }
}
