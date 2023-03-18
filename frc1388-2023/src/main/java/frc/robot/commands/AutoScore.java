// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class AutoScore extends CommandBase {
  private ArmSubsystem m_armSubsystem;
  private GrabberSubsystem m_grabberSubsystem;
  private final double m_wristPosition;
  private final double m_grabberPosition;
  private final double m_mainArmPosition;
  /** Creates a new AutoScore. */
  public AutoScore(double mainArmPosition, double wristPosition, double grabberPosition, ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_armSubsystem = armSubsystem;
    m_grabberSubsystem = grabberSubsystem;
    m_mainArmPosition = mainArmPosition;
    m_wristPosition = wristPosition;
    m_grabberPosition = grabberPosition;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armSubsystem.setPrimaryMotorPosition(0.2);
    m_armSubsystem.setWristMotorPosition(0.4);
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
