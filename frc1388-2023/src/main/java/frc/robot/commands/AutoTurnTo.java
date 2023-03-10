// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADIS16448_IMU.IMUAxis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.GyroSubsystem;

public class AutoTurnTo extends CommandBase {

  private final double m_turnSpeed;
  private final double m_turnAngleSet;
  private GyroSubsystem m_gyroSubsystem;
  private DriveTrainSubsystem m_driveTrainSubsystem;
  private final PIDController m_pidController = new PIDController(AutoConstants.TURN_P_VALUE, 0, 0);

  /** Creates a new AutoTurn. */
  public AutoTurnTo(double turnAngleSet, double turnSpeed, DriveTrainSubsystem driveTrainSubsystem, GyroSubsystem gyroSubsystem) {
    m_turnSpeed = turnSpeed;
    m_turnAngleSet = turnAngleSet;
    m_driveTrainSubsystem = driveTrainSubsystem;
    m_gyroSubsystem = gyroSubsystem;
    //System.out.println("*****************TURNCONSTUCTOR****************************************TURNCONSTRUCTOR*******************");
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrainSubsystem);
    SmartDashboard.putNumber("AutoTurnToSpeed", 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pidController.setTolerance(AutoConstants.TURN_P_TOLERANCE);
    m_pidController.enableContinuousInput(0, 360);
    m_driveTrainSubsystem.setDeadbandZero();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed;
    double turnMinSpeed; 
    double angle = m_gyroSubsystem.getZAngle();
    double rate = m_gyroSubsystem.getZRate();
   
    if (rate > AutoConstants.TURN_MIN_SPEED_THRESHOLD) {
      turnMinSpeed = AutoConstants.TURN_MIN_SPEED_MOVING;
    }
    else {
      turnMinSpeed = AutoConstants.TURN_MIN_SPEED_STOPPED;
    }

    speed = m_pidController.calculate(angle, m_turnAngleSet);
    if (speed > 0) {
      speed = MathUtil.clamp(speed, turnMinSpeed, m_turnSpeed);
    }
    else {
      speed = MathUtil.clamp(speed, -m_turnSpeed, -turnMinSpeed);
    }
   
    SmartDashboard.putNumber("AutoTurnToSpeed", speed);

    //System.out.println("Angle: "+angle+"\tturnSpeed: "+turnSpeed+"\tTurnSetPoint"+m_turnAngleSet);

    m_driveTrainSubsystem.curvatureDrive(0, speed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pidController.reset();
    m_driveTrainSubsystem.curvatureDrive(0, 0, false);
    SmartDashboard.putNumber("AutoTurnToSpeed", 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean finished = (m_pidController.atSetpoint());
    return finished;
  }
}