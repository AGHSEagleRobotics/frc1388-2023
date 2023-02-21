// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.AutoConstants.*;

import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.GyroSubsystem;

public class GoUntilAngle extends CommandBase {
  private double m_angle;
  private DriveTrainSubsystem m_driveTrain;
  private GyroSubsystem m_gyroSubsystem;
  
  /** Creates a new AutoMove. */

  public GoUntilAngle(DriveTrainSubsystem driveTrain, GyroSubsystem gyroSubsystem, double angle) {
    m_driveTrain = driveTrain;
    m_gyroSubsystem = gyroSubsystem;
    m_angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  if (Math.abs(m_gyroSubsystem.getYAngle()) < m_angle) m_driveTrain.constantSpeedDrive(36);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.curvatureDrive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(m_gyroSubsystem.getYAngle()) > m_angle);  
  }
}
