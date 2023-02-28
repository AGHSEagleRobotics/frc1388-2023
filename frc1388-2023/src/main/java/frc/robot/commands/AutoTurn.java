// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class AutoTurn extends CommandBase {
  //private DriveTrainSubsystem m_driveTrainSubsystem;
  private final double m_turnSpeed;
  private final double m_turnAngleSet;

  //private final PIDController m_pidController = new PIDController(AutoConstants.TURN_P_VALUE, 0, 0);
  
  /** Creates a new AutoTurn. */
  public AutoTurn(
    //DriveTrainSubsystem driveTrainSubsystem,
    double turnAngleSet,
    double turnSpeed
  ) {
    //m_driveTrainSubsystem = driveTrainSubsystem;
    m_turnSpeed = turnSpeed;
    m_turnAngleSet = turnAngleSet;
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_driveTrainSubsystem.resetGyro();
    //m_driveTrainSubsystem.setDeadbandZero();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turnSpeed;
    //double angle = m_driveTrainSubsystem.getGyroAngle();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_pidController.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
