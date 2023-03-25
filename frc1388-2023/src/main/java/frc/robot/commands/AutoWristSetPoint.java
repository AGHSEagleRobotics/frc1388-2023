// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class AutoWristSetPoint extends CommandBase {
  public enum WristPositions {
    extend, retract
  }
  private final WristPositions m_wristSetPoint;

  private final ArmSubsystem m_armSubsystem;

  /** Creates a new AutoWristSetPoint. */
  public AutoWristSetPoint(ArmSubsystem armSubsystem, WristPositions setPoint) {
    m_armSubsystem = armSubsystem;
    
    m_wristSetPoint = setPoint;

    addRequirements(m_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
