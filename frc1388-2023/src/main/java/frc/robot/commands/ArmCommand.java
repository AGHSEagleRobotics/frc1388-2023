// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends CommandBase {
  private final ArmSubsystem m_ArmSubsystem;

  private final Supplier<Double> m_opLeftY;
  private final Supplier<Double> m_opRightY;

  /** Creates a new ArmCommand. */
  public ArmCommand(
    ArmSubsystem armSubsystem,
    Supplier<Double> opLeftY,
    Supplier<Double> opRightY
  ) {
    m_ArmSubsystem = armSubsystem;
    m_opLeftY = opLeftY;
    m_opRightY = opRightY;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ArmSubsystem.setMidArmMotor(m_opRightY.get());
    m_ArmSubsystem.setPrimaryMotor(m_opLeftY.get());
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
