// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.GyroSubsystem;

public class FastAutoBalance extends CommandBase {
  public enum BalanceStates {
    going, stopped
  }
  // subsystems
  private DriveTrainSubsystem m_driveTrainSubsystem;
  private GyroSubsystem m_gyroSubsystem;
  
  private double lastAngle = 0.0;
  
  /** Creates a new FastAutoBalance. */
  public FastAutoBalance(DriveTrainSubsystem driveTrainSubsystem, GyroSubsystem gyroSubsystem) {
    m_driveTrainSubsystem = driveTrainSubsystem;
    m_gyroSubsystem = gyroSubsystem;
    addRequirements(m_driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = m_gyroSubsystem.getYAngle();
    if ((Math.abs(angle) > Math.abs(lastAngle) - 0.12) && (Math.abs(angle) > 2.5)) {
      double speed = (Math.pow(angle, 4) * 0.0005) + 3.0;
      speed = MathUtil.clamp(speed, -24.0, 24.0);
      m_driveTrainSubsystem.constantSpeedDrive(Math.copySign(speed, -angle));
    } else {
      m_driveTrainSubsystem.tankDrive(0, 0);
    }
    lastAngle = angle;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
