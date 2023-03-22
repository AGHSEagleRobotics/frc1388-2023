// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.GrabberConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.GrabberSubsystem.GrabberPosition;

public class GrabberCommand extends CommandBase {
  private enum Direction {
    in, out
  }
  private Direction m_grabberDirection;

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

  // @Deprecated
  // public void setGrabber(GrabberPosition setPoint) {
  //   m_grabberSubsystem.setGrabberPresetPosition(setPoint);
  // }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    String state;

    if (m_grabberSubsystem.getGrabberEncoder() > GrabberConstants.GRABBER_MAX_AT_FULL_ARM &&
        (m_ArmSubsystem.getPrimaryArmPosition() > ArmConstants.ARM_MAX_EXTEND_LOW)
        && (m_ArmSubsystem.getPrimaryArmPosition() < ArmConstants.ARM_MAX_EXTEND_HIGH)) {
      m_grabberSubsystem.setGrabberMotor(GrabberConstants.GRABBER_POWER_IN);
      state = "arm too high, pulling grabber in ";
    } else if ((m_opLeftTrigger.get() > GrabberConstants.GRABBER_GOOD_ENOUGH_SQUEEZE)
        && ((m_grabberSubsystem.getGrabberEncoder() < GrabberConstants.GRABBER_CLOSE_MAX_AT_FULL_ARM)
            || ((m_ArmSubsystem.getPrimaryArmPosition() < ArmConstants.ARM_MAX_EXTEND_LOW)
                || (m_ArmSubsystem.getPrimaryArmPosition() > ArmConstants.ARM_MAX_EXTEND_HIGH)))) {
      m_grabberSubsystem.setGrabberMotor(GrabberConstants.GRABBER_POWER_OUT);
      m_grabberDirection = Direction.out;
      state = "left trigger out";
    } else if (m_opRightTrigger.get() > GrabberConstants.GRABBER_GOOD_ENOUGH_SQUEEZE) {
      m_grabberSubsystem.setGrabberMotor(GrabberConstants.GRABBER_POWER_IN);
      m_grabberDirection = Direction.in;
      state = "right trigger in";
    } else {
      if (m_grabberDirection == Direction.in) {
        m_grabberSubsystem.setGrabberMotor(GrabberConstants.GRABBER_LOW_POWER_IN);
      }
      m_grabberSubsystem.setGrabberMotor(0);
      state = "default, grabber at 0";
    }

    SmartDashboard.putString("current grabber state   ", state);
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
