// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.WristSubsystem;

public class AutoWristSetPoint extends CommandBase {
private double SWITCH_TO_LOW_POWER = 1.0;
private double STOP_TIME = 2.0;

  public enum WristPositions {
    extend, retract
  }
  private final WristPositions m_wristSetPoint;

  private final WristSubsystem m_wristSubsystem;
  
  private final Timer m_timer;

  /** Creates a new AutoWristSetPoint. */
  public AutoWristSetPoint(WristSubsystem wristSubsystem, WristPositions setPoint) {
    m_wristSubsystem = wristSubsystem;

    m_wristSetPoint = setPoint;

    addRequirements(m_wristSubsystem);
    m_timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_wristSetPoint == WristPositions.extend) {
      if (m_timer.get() < SWITCH_TO_LOW_POWER) {
        m_wristSubsystem.setWristMotorPower(-1.0 * ArmConstants.WRIST_POWER_SCALE_FACTOR);
      } else {
        m_wristSubsystem.setWristMotorPower(-0.25 * ArmConstants.WRIST_POWER_SCALE_FACTOR);
      }
    } else if(m_wristSetPoint == WristPositions.retract) {
      if (m_timer.get() < SWITCH_TO_LOW_POWER) {
        m_wristSubsystem.setWristMotorPower(1.0 * ArmConstants.WRIST_POWER_SCALE_FACTOR);
      } else {
        m_wristSubsystem.setWristMotorPower(0.25 * ArmConstants.WRIST_POWER_SCALE_FACTOR);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_timer.get() >= STOP_TIME);
  }
}
