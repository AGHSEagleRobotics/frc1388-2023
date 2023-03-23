// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.GrabberConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class ArmCommand extends CommandBase {
  private enum WristPosition {
    flat, stowed
  }
  private WristPosition m_wristPosition = WristPosition.stowed;

  private final ArmSubsystem m_ArmSubsystem;
  private final GrabberSubsystem m_grabberSubsystem;

  private final Supplier<Double> m_opLeftY;
  private final Supplier<Double> m_opRightY;
  // private final Supplier<Double> m_opLTrigger;
  // private final Supplier<Double> m_opRTrigger;

  /** Creates a new ArmCommand. */
  public ArmCommand(
    ArmSubsystem armSubsystem,
    GrabberSubsystem grabberSubsystem,
    Supplier<Double> opLeftY,
    Supplier<Double> opRightY
  ) {
    m_ArmSubsystem = armSubsystem;
    m_grabberSubsystem = grabberSubsystem;
    m_opLeftY = opLeftY;
    m_opRightY = opRightY;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ArmSubsystem);
  }

  public void toggleWristPosition() {
    if (m_wristPosition == WristPosition.stowed) m_wristPosition = WristPosition.flat;
    if (m_wristPosition == WristPosition.flat) m_wristPosition = WristPosition.stowed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ArmSubsystem.setWristMotorPower(-0.3 * m_opLeftY.get()); //XXX scaling]\[]
            m_ArmSubsystem.setPrimaryArmMotorPower(ArmConstants.ARM_POWER_SCALE_FACTOR * -m_opRightY.get());
          }
          else
          {
            m_ArmSubsystem.setPrimaryArmMotorPower(0);
          }
    } else {
      m_ArmSubsystem.setPrimaryArmMotorPower(ArmConstants.ARM_POWER_SCALE_FACTOR * -m_opRightY.get());
    }

    // if (m_grabberSubsystem.getGrabberEncoder() > GrabberConstants.GRABBER_MAX_AT_FULL_ARM && 
    //     (m_ArmSubsystem.getPrimaryArmPosition() > ArmConstants.ARM_MAX_EXTEND_LOW && m_ArmSubsystem.getPrimaryArmPosition() < ArmConstants.ARM_MAX_EXTEND_HIGH)) {
    //       if (-m_opRightY.get() < 0 )
    //       {
    //         m_ArmSubsystem.setPrimaryMotorPower(ArmConstants.ARM_POWER_SCALE_FACTOR * -m_opRightY.get());
    //       }
    //       else
    //       {
    //         m_ArmSubsystem.setPrimaryMotorPower(0);
    //       }
    // } else {
    //   m_ArmSubsystem.setPrimaryMotorPower(ArmConstants.ARM_POWER_SCALE_FACTOR * -m_opRightY.get());
    // }


    //test
    // if (m_wristPosition == WristPosition.flat) m_ArmSubsystem.parallelArmSet(m_opRightY.get());
    // if (m_wristPosition == WristPosition.stowed) m_ArmSubsystem.stowedArmSet(m_opRightY.get());
    // endtest
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ArmSubsystem.setPrimaryArmMotorPower(0);
    m_ArmSubsystem.setWristMotorPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
