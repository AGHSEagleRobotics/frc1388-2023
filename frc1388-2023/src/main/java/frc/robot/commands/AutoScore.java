// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.GrabberConstants;
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
    //starting position: wrist is as far up as it can be against the arm, arm is down. Within frame position
    m_grabberSubsystem.setGrabberMotor(GrabberConstants.GRABBER_LOW_POWER_IN); //makes sure the cone is held - POWER
    if ( m_armSubsystem.getPrimaryArmPosition() < ArmConstants.ARM_MAX_EXTEND_HIGH )
    {
      m_armSubsystem.setPrimaryArmMotorPower(0.35); 
    }
    if ( m_armSubsystem.getWristPosition() < ArmConstants.WRIST_POSITION_SCORE )
    {
      m_armSubsystem.setWristMotorPower(-0.05); //starts from max top position, needs to go down to score
    }
    m_grabberSubsystem.setGrabberMotor(0.3); //POWER //Move this 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.setPrimaryArmMotorPower(0);
    m_armSubsystem.setWristMotorPosition(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
