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
import frc.robot.subsystems.GyroSubsystem;

public class AutoMove extends CommandBase {
  
  private final double m_setPoint;
  private final double m_speed;
  private final double m_angleSet;
  private final boolean m_isSteering;

  private GyroSubsystem m_gyroSubsystem;
  private DriveTrainSubsystem m_driveTrainSubsystem;

  private final PIDController m_pidTurn = new PIDController(TURN_P_VALUE, 0, 0);
  
  private final PIDController m_pidMove = new PIDController(CURVE_P_VALUE, 0, 0);

  /** Creates a new AutoMove. 
   * @setPoint distance to travel in inches
   * @speed motor power
  */
  public AutoMove(double setPoint, double speed, double angle,  DriveTrainSubsystem driveTrainSubsystem, GyroSubsystem gyroSubsystem) {
    m_setPoint = setPoint;
    m_speed = speed;
    m_angleSet = angle;
    m_isSteering = true;

    m_gyroSubsystem = gyroSubsystem;
    m_driveTrainSubsystem = driveTrainSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(driveTrainSubsystem);

    //Setting PID control tolerance
    m_pidMove.setTolerance(MOVE_P_TOLERANCE); //change P tolerance?
  }

  public AutoMove( double setPoint, double speed, DriveTrainSubsystem driveTrainSubsystem, GyroSubsystem gyroSubsystem) {
    m_setPoint = setPoint;
    m_speed = speed;
    m_angleSet = -1;
    m_isSteering = false;
    m_driveTrainSubsystem = driveTrainSubsystem;
    m_gyroSubsystem = gyroSubsystem;

    addRequirements(driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DataLogManager.log("setpoint: " + m_setPoint + "   " + "speed: " + m_speed + "   " + "curve: " + m_angleSet);
    SmartDashboard.putNumber("AutoSpeed", m_speed);
    m_driveTrainSubsystem.resetEncoders();
    m_driveTrainSubsystem.setDeadbandZero();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed;
    double curve;
    double angle = m_gyroSubsystem.getZAngle();

    double averageEncoderDistance = m_driveTrainSubsystem.getAverageEncoderDistance();


    speed = m_pidMove.calculate(averageEncoderDistance, m_setPoint);
    speed = MathUtil.clamp(speed, -m_speed, m_speed);
    speed += Math.copySign(MOVE_F_VALUE, speed);

    if( m_isSteering == false){
      curve = 0;
    }
    else{
      curve = m_pidTurn.calculate(angle, m_angleSet);
      curve = MathUtil.clamp(curve, -CURVE_MAX, CURVE_MAX);
    }
    SmartDashboard.putNumber("AutoMoveSpeed", speed);
    SmartDashboard.putNumber("AutoMoveCurve", curve);

    m_driveTrainSubsystem.curvatureDrive(speed, curve, false); 
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pidMove.reset();
    m_driveTrainSubsystem.curvatureDrive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return finished;
    boolean finished = (m_pidMove.atSetpoint());
    return finished;
  }
}