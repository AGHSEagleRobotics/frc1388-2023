// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class AutoMoveSecondaryArm extends CommandBase {
  
  private final ArmSubsystem m_armSubsystem;

  private final double m_secondaryArmSetPoint;

  /** Creates a new AutoMoveSecondaryArm. */
  public AutoMoveSecondaryArm(ArmSubsystem armSubsystem, double secondaryArmSetPoint) {
    m_armSubsystem  = armSubsystem;

    m_secondaryArmSetPoint = secondaryArmSetPoint;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armSubsystem.setWristMotorPosition(m_secondaryArmSetPoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.setWristMotorPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(m_armSubsystem.getWristPosition() - m_secondaryArmSetPoint) < ArmConstants.DEADBAND);
  }
}
