// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.RumbleSubsystem;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.RumbleConstants;

public class DriveTrainCommand extends CommandBase {
  private final DriveTrainSubsystem m_driveTrain;

  public enum Side {
    left, right
  }

  // driver controller
  public enum Direction {
    forwards, reverse
  }
  private Direction m_direction = Direction.forwards;

  private boolean m_quickTurn = true;
  private Supplier<Double> m_driveLeftStickYAxis;
  private Supplier<Double> m_driveRightStickXAxis;
  private Supplier<Boolean> m_driveRightStickButton;
  private boolean m_lastStick = false;
  private RumbleSubsystem m_precisionRumble;
  private boolean m_lastRightStickButton = false;
  private boolean m_precisionMode = false;

  // op controller
  private Supplier<Double> m_opLeftStickYAxis;
  private Supplier<Double> m_opRightStickXAxis;
  
  /** Creates a new DriveTrainCommand. */
  public DriveTrainCommand(
    DriveTrainSubsystem driveTrain,
    Supplier<Double> driveLeftStickYAxis, 
    Supplier<Double> driveRightStickXAxis,
    Supplier<Boolean> driveRightStickButton,

    Supplier<Double> opLeftY,
    Supplier<Double> opRightX,

    RumbleSubsystem precisionrumble
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);

    m_driveTrain = driveTrain;
    m_driveLeftStickYAxis = driveLeftStickYAxis;
    m_driveRightStickXAxis = driveRightStickXAxis;
    m_driveRightStickButton = driveRightStickButton;

    m_opLeftStickYAxis = opLeftY;
    m_opRightStickXAxis =  opRightX;

    m_precisionRumble = precisionrumble;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double speed = -m_driveLeftStickYAxis.get();
    speed = MathUtil.applyDeadband(speed, 0.03);
    speed = Math.tan(speed * Math.atan(DriveTrainConstants.DRIVE_SCALING_CONSTANT)) / DriveTrainConstants.DRIVE_SCALING_CONSTANT; // posable scaling curve idea, math.atan(5) could be precalculated, or this entire function could be precalculated.

    double  rotation = -m_driveRightStickXAxis.get();
    rotation = MathUtil.applyDeadband(rotation, 0.03);
    rotation = Math.tan(rotation * Math.atan(DriveTrainConstants.DRIVE_SCALING_CONSTANT)) / DriveTrainConstants.DRIVE_SCALING_CONSTANT; // posable scaling curve idea, math.atan(5) could be precalculated, or this entire function could be precalculated.\

    // maybe add this later
    // double opSpeed = m_opLeftStickYAxis.get();
    // opSpeed = MathUtil.applyDeadband(opSpeed, 0.1);
    
    // double opRotation = m_opRightStickXAxis.get();
    // opRotation = MathUtil.applyDeadband(opRotation, 0.1);
    
    if (m_driveRightStickButton.get() && !m_lastStick) {
      m_quickTurn = !m_quickTurn;
    }

    boolean rightStickButton = m_driveRightStickButton.get();
    if (rightStickButton && !m_lastRightStickButton ) {
      m_precisionMode = !m_precisionMode;
      if( m_precisionMode){
        m_precisionRumble.rumblePulse(RumbleConstants.RumbleSide.RIGHT);
      }
      else{
        m_precisionRumble.rumblePulse(RumbleConstants.RumbleSide.LEFT); 
      }
    }
    m_lastRightStickButton = rightStickButton;
    
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

    //test code, remove later if no one is using
    // SmartDashboard.putString("direction ", m_direction.name());
    // SmartDashboard.putNumber("speed ", speed);
    // SmartDashboard.putNumber("rotation", rotation);
  }

  public void setDirection(Direction direction) {
    m_direction = direction;
  }

  public void turnSlow(Side direction) {
    if (direction == Side.left) m_driveTrain.tankDrive(-0.3, 0.3);
    if (direction == Side.right) m_driveTrain.tankDrive(0.3, -0.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
