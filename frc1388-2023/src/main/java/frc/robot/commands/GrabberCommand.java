// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.GrabberSubsystem.GrabberPosition;

public class GrabberCommand extends CommandBase {
  private final GrabberSubsystem m_grabberSubsystem;
  private final ArmSubsystem m_ArmSubsystem;

  private final Supplier<Double> m_opLeftTrigger;
  private final Supplier<Double> m_opRightTrigger;
  /** Creates a new GrabberCommand. */
  public GrabberCommand(
    GrabberSubsystem grabberSubsystem,
    ArmSubsystem armSubsystem,
    Supplier<Double> opLeftTrigger,
    Supplier<Double> opRightTrigger
  ) {
    m_grabberSubsystem = grabberSubsystem;
    m_ArmSubsystem = armSubsystem;

    m_opLeftTrigger = opLeftTrigger;
    m_opRightTrigger = opRightTrigger;

    addRequirements(m_grabberSubsystem);
  }

  @Deprecated
  public void setGrabber(GrabberPosition setPoint) {
    m_grabberSubsystem.setGrabberPresetPosition(setPoint);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if ((m_ArmSubsystem.getPrimaryArmPosition() > 0.2) && (m_ArmSubsystem.getWristPosition() < 0.3)) {
    //   m_grabberSubsystem.setGrabberPosition(10);
    // } else {
    // }
    m_grabberSubsystem.setGrabberMotor(0);
    if(m_opLeftTrigger.get() > 0.3) {
      // m_grabberSubsystem.setGrabberPosition(GrabberPosition.open);
      m_grabberSubsystem.setGrabberMotor(0.25);
    } else if(m_opRightTrigger.get() > 0.3) {
      // m_grabberSubsystem.setGrabberPosition(GrbabberPosition.closed);
      m_grabberSubsystem.setGrabberMotor(-0.25);
    }
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
