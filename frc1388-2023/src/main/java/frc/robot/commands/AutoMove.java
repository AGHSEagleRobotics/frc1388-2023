// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController; 
import static frc.robot.Constants.AutoConstants.*;

public class AutoMove extends CommandBase {

  //private DriveTrainSubsystem m_driveTrainSubsystem;
  private final double m_setPoint; //TODO once encoder is tested in AutoMetod, change back
  private final double m_speed;
  //private final double m_curve;

  private final PIDController m_pidController = new PIDController(MOVE_P_VALUE, 0, 0);

  /** Creates a new AutoMove. */
  public AutoMove(
    //DriveTrainSubsystem driveTrainSubsystem,
    double setPoint,
    //setPoint being the goal
    double speed
    //,double curve
  ) 
  {
    //m_driveTrainSubsystem = driveTrainSubsystem;
    m_setPoint = setPoint;
    m_speed = speed;
    //m_curve = curve;
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(driveTrainSubsystem);
  }

  /* 
  public AutoMove( double speed )
  {
    m_speed = speed;
  }
  */

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double speed;
    speed = MathUtil.clamp( m_speed, -m_speed, m_speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_pidController.reset();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean finished = (m_pidController.atSetpoint());
    return finished;
  }
}
