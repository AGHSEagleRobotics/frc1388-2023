// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class ArmCommand extends CommandBase {

  private final ArmSubsystem m_armSubsystem;

  private final Supplier<Double> m_opLeftY;
  private final Supplier<Double> m_opRightY;

  /** Creates a new ArmCommand. */
  public ArmCommand(
    ArmSubsystem armSubsystem,
    Supplier<Double> opLeftY,
    Supplier<Double> opRightY
  ) {
    m_armSubsystem = armSubsystem;
    m_opLeftY = opLeftY;
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
    m_armSubsystem.setWristMotorPower(ArmConstants.WRIST_POWER_SCALE_FACTOR * -m_opLeftY.get());
    m_armSubsystem.setPrimaryArmMotorPower(ArmConstants.ARM_POWER_SCALE_FACTOR * -m_opRightY.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.setPrimaryArmMotorPower(0);
    m_armSubsystem.setWristMotorPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
