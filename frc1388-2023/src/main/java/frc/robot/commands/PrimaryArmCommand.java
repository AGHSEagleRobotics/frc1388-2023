// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.PrimaryArmSubsystem;

public class PrimaryArmCommand extends CommandBase {

  private final PrimaryArmSubsystem m_armSubsystem;

  private final Supplier<Double> m_opRightY;

  /** Creates a new ArmCommand. */
  public PrimaryArmCommand(
    PrimaryArmSubsystem armSubsystem,
    Supplier<Double> opRightY
  ) {
    m_armSubsystem = armSubsystem;
    m_opRightY = opRightY;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armSubsystem.setPrimaryArmMotorPower(ArmConstants.ARM_POWER_SCALE_FACTOR * -m_opRightY.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.setPrimaryArmMotorPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
