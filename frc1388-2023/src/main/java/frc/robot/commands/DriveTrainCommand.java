// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.FollowerType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveTrainCommand extends CommandBase {
  private final DriveTrain m_driveTrain;

  // driver controller
  public enum Direction {
    forwards, reverse;
  }
  private Direction m_direction = Direction.forwards;

  private boolean m_quickTurn;
  private Supplier<Double> m_driveLeftStickYAxis;
  private Supplier<Double> m_driveRightStickXAxis;
  private Supplier<Boolean> m_driveRightStickButton;
  private boolean m_lastStick = false;

  // op controller
  private Supplier<Double> m_opLeftStickYAxis;
  private Supplier<Double> m_opRightStickXAxis;

  /** Creates a new DriveTrainCommand. */
  public DriveTrainCommand(
    DriveTrain driveTrain,
    Supplier<Double> driveLeftStickYAxis, 
    Supplier<Double> driveRightStickXAxis,
    Supplier<Boolean> rightStickButton,

    Supplier<Double> opLeftY,
    Supplier<Double> opRightX
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);

    m_driveTrain = driveTrain;
    m_driveLeftStickYAxis = driveLeftStickYAxis;
    m_driveRightStickXAxis = driveRightStickXAxis;
    m_driveRightStickButton = rightStickButton;

    m_opLeftStickYAxis = opLeftY;
    m_opRightStickXAxis =  opRightX;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double speed = -m_driveLeftStickYAxis.get();
    // speed = MathUtil.applyDeadband(speed, 0.03);
    speed = Math.tan(speed * Math.atan(5)) / 5; // posable scaling curve idea, math.atan(5) could be precalculated, or this entire function could be precalculated.

    double  rotation = -m_driveRightStickXAxis.get();
    // rotation = MathUtil.applyDeadband(rotation, 0.03);
    rotation = Math.tan(rotation * Math.atan(5)) / 5; // posable scaling curve idea, math.atan(5) could be precalculated, or this entire function could be precalculated.\

    // maybe add this later
    // double opSpeed = m_opLeftStickYAxis.get();
    // opSpeed = MathUtil.applyDeadband(opSpeed, 0.1);
    
    // double opRotation = m_opRightStickXAxis.get();
    // opRotation = MathUtil.applyDeadband(opRotation, 0.1);
    
    if (m_driveRightStickButton.get() && !m_lastStick) {
      m_quickTurn = !m_quickTurn;
    }
    
    if (m_direction == Direction.forwards) {
      m_driveTrain.curvatureDrive(speed, rotation, m_quickTurn);
    } else {
      m_driveTrain.curvatureDrive(-speed, rotation, m_quickTurn);
    }
    
    // maybe add this later
    // if (opSpeed != 0) {
    //   m_driveTrain.constantSpeedDrive(12.0 * m_opLeftStickYAxis.get());
    // }
    // if (opRotation != 0) {
    //   m_driveTrain.tankDrive(0.5 * opRotation, -0.5 * opRotation);
    // }
    
    m_lastStick = m_driveRightStickButton.get();
    SmartDashboard.putString("direction ", m_direction.name());
    SmartDashboard.putNumber("speed ", speed);
  }

  public void setDirection(Direction direction) {
    m_direction = direction;
    System.out.println(direction.name());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
