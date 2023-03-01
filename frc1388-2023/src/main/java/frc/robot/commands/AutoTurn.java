// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.GyroSubsystem;

public class AutoTurn extends CommandBase {

  private DriveTrainSubsystem m_driveTrainSubsystem;
  private final double m_turnSpeed;
  private final double m_turnAngleSet;
  private GyroSubsystem m_gyroSubsystem;
  private final PIDController m_pidController = new PIDController(AutoConstants.TURN_P_VALUE, 0, 0);

  /** Creates a new AutoTurn. */
  public AutoTurn(DriveTrainSubsystem driveTrainSubsystem, double turnSpeed, double turnAngleSet, GyroSubsystem gyroSubsystem) {
    m_driveTrainSubsystem = driveTrainSubsystem;
    m_turnSpeed = turnSpeed;
    m_turnAngleSet = turnAngleSet;
    m_gyroSubsystem = gyroSubsystem;
    //System.out.println("*****************TURNCONSTUCTOR****************************************TURNCONSTRUCTOR*******************");
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrainSubsystem);

    m_pidController.setTolerance(AutoConstants.TURN_P_TOLERANCE);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_gyroSubsystem.resetZAngle();
    m_driveTrainSubsystem.setDeadbandZero();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turnSpeed;
    double angle = m_gyroSubsystem.getZAngle();


    turnSpeed = m_pidController.calculate(angle, m_turnAngleSet);
    turnSpeed = MathUtil.clamp(turnSpeed, -m_turnSpeed, m_turnSpeed);

    //System.out.println("Angle: "+angle+"\tturnSpeed: "+turnSpeed+"\tTurnSetPoint"+m_turnAngleSet);

    m_driveTrainSubsystem.curvatureDrive(0, turnSpeed, true);
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
    boolean finished = (m_pidController.atSetpoint());
    return finished;
  }
}