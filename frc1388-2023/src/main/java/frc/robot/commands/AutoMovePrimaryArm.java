// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.print.attribute.standard.PrinterMessageFromOperator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class AutoMovePrimaryArm extends CommandBase {

  private final ArmSubsystem m_armSubsystem;

  private final double m_primaryArmSetPoint;

  /** Creates a new AutoMoveArm. */
  public AutoMovePrimaryArm(ArmSubsystem armSubsystem, double primaryArmSetPoint) {
    m_armSubsystem = armSubsystem;

    m_primaryArmSetPoint = primaryArmSetPoint;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armSubsystem.setPrimaryMotorPosition(m_primaryArmSetPoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.setPrimaryMotorPower(0);
    m_armSubsystem.setWristMotorPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
