// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class AutoMoveWrist extends CommandBase {
  
  private final WristSubsystem m_requirements;

  private final double m_wristSetPoint;
  private boolean m_atSetPoint = false;

  /** Creates a new AutoMoveSecondaryArm. */
  public AutoMoveWrist(WristSubsystem wristSubsystem, double wristSetPoint) {
    m_requirements  = wristSubsystem;

    m_wristSetPoint = wristSetPoint;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_atSetPoint = m_requirements.setWristMotorPosition(m_wristSetPoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_requirements.setWristMotorPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_atSetPoint;
  }
}
