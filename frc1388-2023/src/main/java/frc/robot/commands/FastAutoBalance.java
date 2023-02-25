// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import org.opencv.core.Mat;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.GyroSubsystem;

public class FastAutoBalance extends CommandBase {
  public enum BalanceStates {
    going, stopped
  }
  private BalanceStates m_balanceState = BalanceStates.going;

  // subsystems
  private DriveTrainSubsystem m_driveTrainSubsystem;
  private GyroSubsystem m_gyroSubsystem;


  ArrayList<Double> averageAngles = new ArrayList<Double>();

  private double lastAngle = 0.0;
  
  /** Creates a new FastAutoBalance. */
  public FastAutoBalance(DriveTrainSubsystem driveTrainSubsystem, GyroSubsystem gyroSubsystem) {
    m_driveTrainSubsystem = driveTrainSubsystem;
    m_gyroSubsystem = gyroSubsystem;

    averageAngles.add(0.0);
    averageAngles.add(0.0);
    // averageAngles.add(0.0);
    // Use addRequirements() here to declare subsystem dependencies.\
    addRequirements(m_driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = m_gyroSubsystem.getYAngle();

    // averageAngles.remove(0);
    // averageAngles.add(angle);

    // double currentAverage = (averageAngles.get(0) + averageAngles.get(1)) / 3.0;

    //test=============
    // SmartDashboard.putNumber("balance angle ", angle);
    // SmartDashboard.putNumber("average angle ", currentAverage);
    // SmartDashboard.putNumber("0 angle ", averageAngles.get(0));
    // SmartDashboard.putNumber("1 angle ", averageAngles.get(1));
    // SmartDashboard.putNumber("2 angle ", averageAngles.get(2));
    //endtest=============

    SmartDashboard.putString("balance state  ", m_balanceState.name());

    if ((Math.abs(angle) > Math.abs(lastAngle) - 0.15) && (Math.abs(angle) > 2.5)) {
      m_balanceState = BalanceStates.going;
      double speed = (Math.pow(angle, 4) * 0.0005) + 3.0;
      speed = MathUtil.clamp(speed, -24.0, 24.0);
      m_driveTrainSubsystem.constantSpeedDrive(Math.copySign(speed, -angle));

    } else {
      m_balanceState = BalanceStates.stopped;
      m_driveTrainSubsystem.tankDrive(0, 0);
    }

    // averageAngles.remove(0);
    // averageAngles.add(angle);\
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
