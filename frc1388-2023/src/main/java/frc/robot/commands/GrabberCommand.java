// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GrabberConstants;
import frc.robot.subsystems.GrabberSubsystem;

public class GrabberCommand extends CommandBase {
  private enum Direction {
    in, out, stopped
  }
  private Direction m_grabberDirection = Direction.stopped;

  private final GrabberSubsystem m_grabberSubsystem;

  private final Supplier<Double> m_opLeftTrigger;
  private final Supplier<Double> m_opRightTrigger;

  /** Creates a new GrabberCommand. */
  public GrabberCommand(
    GrabberSubsystem grabberSubsystem,

    Supplier<Double> opLeftTrigger,
    Supplier<Double> opRightTrigger
  ) {
    m_grabberSubsystem = grabberSubsystem;

    m_opLeftTrigger = opLeftTrigger;
    m_opRightTrigger = opRightTrigger;

    addRequirements(m_grabberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_opLeftTrigger.get() > GrabberConstants.GRABBER_GOOD_ENOUGH_SQUEEZE) {
      m_grabberDirection = Direction.out;
      m_grabberSubsystem.setGrabberMotor(GrabberConstants.GRABBER_POWER_OUT);
    } else if (m_opRightTrigger.get() > GrabberConstants.GRABBER_GOOD_ENOUGH_SQUEEZE) {
      m_grabberDirection = Direction.in;
      m_grabberSubsystem.setGrabberMotor(GrabberConstants.GRABBER_POWER_IN);
    } else if (m_grabberDirection == Direction.in) {
      m_grabberSubsystem.setGrabberMotor(GrabberConstants.GRABBER_LOW_POWER_IN);
    } else {
      m_grabberDirection = Direction.stopped;
      m_grabberSubsystem.setGrabberMotor(0);
    }

    SmartDashboard.putString("grabber direction", m_grabberDirection.name());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_grabberDirection = Direction.stopped;
    m_grabberSubsystem.setGrabberMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
