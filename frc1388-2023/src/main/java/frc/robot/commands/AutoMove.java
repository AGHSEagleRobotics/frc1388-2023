// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.AutoConstants.*;

import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class AutoMove extends CommandBase {
  
  private final double m_setPoint;
  private final double m_speed;
  private final double m_curve;
  private DriveTrainSubsystem m_driveTrainSubsystem;
  
  private final PIDController m_pidController = new PIDController(MOVE_P_VALUE, 0, 0);

  /** Creates a new AutoMove. 
   * @setPoint distance to travel in inches
   * @speed motor power
  */
  public AutoMove(double setPoint, double speed, double curve, DriveTrainSubsystem driveTrainSubsystem) {
    m_setPoint = setPoint;
    m_speed = speed;
    m_curve = curve;
    m_driveTrainSubsystem = driveTrainSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(driveTrainSubsystem);

    //Setting PID control tolerance
    m_pidController.setTolerance(MOVE_P_TOLERANCE); //change P tolerance?
  }

  public AutoMove( double setPoint, double speed, DriveTrainSubsystem driveTrainSubsystem) {
    this(setPoint, speed, 0.0, driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DataLogManager.log("setpoint: " + m_setPoint + "   " + "speed: " + m_speed + "   " + "curve: " + m_curve);
    SmartDashboard.putNumber("AutoSpeed", m_speed);
    m_driveTrainSubsystem.resetEncoders();
    m_driveTrainSubsystem.setDeadbandZero();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed;
    double averageEncoderDistance = m_driveTrainSubsystem.getAverageEncoderDistance();
    DataLogManager.log("average encoder distance" + averageEncoderDistance);

    speed = m_pidController.calculate(averageEncoderDistance, m_setPoint);
    speed = MathUtil.clamp(speed, -m_speed, m_speed);
    // speed += Math.copySign(MOVE_F_VALUE, speed);
    if (speed > 0) {
      MathUtil.clamp(speed, AutoConstants.MOVE_MIN_SPEED, m_speed);
    }
    else {
      MathUtil.clamp(speed, -m_speed, -AutoConstants.MOVE_MIN_SPEED);
    }
    m_driveTrainSubsystem.curvatureDrive(speed, m_curve, false); 
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pidController.reset();
    m_driveTrainSubsystem.curvatureDrive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return finished;
    boolean finished = (m_pidController.atSetpoint());
    return finished;
  }
}