// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveTrainCommand extends CommandBase {
  private final DriveTrain m_driveTrain;
  
  private Supplier<Double> m_driveLeftStickYAxis;
  private Supplier<Double> m_driveRightStickYAxis;
  private Supplier<Double> m_driveRightStickXAxis;
  /** Creates a new DriveTrainCommand. */
  public DriveTrainCommand( DriveTrain driveTrain, Supplier<Double> driveLeftStickYAxis, 
  Supplier<Double> driveRightStickYAxis, Supplier<Double> driveRightStickXAxis ) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);

    m_driveTrain = driveTrain;
    m_driveLeftStickYAxis = driveLeftStickYAxis; 
    m_driveRightStickXAxis = driveRightStickXAxis;
    m_driveRightStickYAxis = driveRightStickYAxis;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = -m_driveLeftStickYAxis.get();
    double rotation = m_driveRightStickXAxis.get();
    m_driveTrain .arcadeDrive(speed, rotation);
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
