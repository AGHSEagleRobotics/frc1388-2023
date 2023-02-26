// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.GyroSubsystem;

public class AutoBalance extends CommandBase {

  private DriveTrainSubsystem m_driveTrainSubsystem;
  private GyroSubsystem m_gyroSubsystem;

  private class Constants {
    private static final double ANGLE_BALANCED = 2.0;
    private static final double ANGLE_STEEP = 10.0;

    private static final double CLIMB_SPEED = 24.0;

    private static final double CREEP_SPEED = 12.0;
    private static final int CREEP_WAIT_ITERATIONS = 5;
    
    private static final double BACK_UP_SPEED = 12.0;
    private static final double BACK_UP_DISTANCE = 1.0;
    private static final int BACK_UP_WAIT_ITERATIONS = 5;

    private static final double DECREASING_ANGLE_THRESHOLD = 1.0;
  }

  private enum BalanceStates {
    idle,
    balanced,
    climb,
    creep,
    backUp,
    wait
  }

  private BalanceStates m_balanceState = BalanceStates.idle;

  private double m_lastAngle = 0;

  private double m_angleDecrease = 0;   // Consecutively decreasing angle amount
  private double m_targetDistance;
  private double m_moveSpeed;
  private int m_waitTime = 0;

  /** Constructor */
  public AutoBalance(DriveTrainSubsystem driveTrainSubsystem, GyroSubsystem gyroSubsystem) {
    m_driveTrainSubsystem = driveTrainSubsystem;
    m_gyroSubsystem = gyroSubsystem;

    addRequirements(driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set the starting state
    m_balanceState = BalanceStates.balanced;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // manage variables related to the robot pitch angle
    double currentAngle = m_gyroSubsystem.getYAngle();
    double absAngle = Math.abs(currentAngle);
    
    // determine which way is uphill (positive is forward, negative is reverse)
    double uphill = Math.signum(-currentAngle);

    // accumulate the consecutive decreasing angle
    double incrementalDecrease = Math.abs(m_lastAngle) - Math.abs(currentAngle);
    if (incrementalDecrease > 0) {
      m_angleDecrease += incrementalDecrease;
    } else {
      m_angleDecrease = 0;
    }

    // State machine
    SmartDashboard.putString("balanceState", m_balanceState.toString());

    switch (m_balanceState) {
      case balanced:
        // Don't move
        m_driveTrainSubsystem.stop();

        // Check to see if we're unbalanced
        if (absAngle > Constants.ANGLE_STEEP) {
          m_balanceState = BalanceStates.climb;
        } else if (absAngle > Constants.ANGLE_BALANCED) {
          m_balanceState = BalanceStates.creep;
        }
        break;

      case climb:
        // climb speed
        m_driveTrainSubsystem.constantSpeedDrive(uphill * Constants.CLIMB_SPEED);

        // If the angle is decreasing, back up a bit to avoid overbalancing
        if (m_angleDecrease >= Constants.DECREASING_ANGLE_THRESHOLD) {
          m_moveSpeed = -uphill * Constants.BACK_UP_SPEED;
          m_targetDistance = m_driveTrainSubsystem.getLeftEncoderDistance()
              + (-uphill * Constants.BACK_UP_DISTANCE);
          m_driveTrainSubsystem.stop();         // This resets the constantSpeedDrive ramp to zero
          m_balanceState = BalanceStates.backUp;
        }
        break;

      case backUp:
        // move until the specified distance has been reached
        m_driveTrainSubsystem.constantSpeedDrive(m_moveSpeed);

        // if target distance is reached, wait a bit
        double position = m_driveTrainSubsystem.getLeftEncoderDistance();
        if ((m_moveSpeed > 0) && (position > m_targetDistance)
            || (m_moveSpeed < 0) && (position < m_targetDistance)) {
          m_waitTime = Constants.BACK_UP_WAIT_ITERATIONS;
          m_balanceState = BalanceStates.wait;
        }
        break;

      case creep:
        // Move uphill slowly
        m_driveTrainSubsystem.constantSpeedDrive(uphill * Constants.CREEP_SPEED);

        // If the angle is decreasing, stop for a bit
        if (m_angleDecrease >= Constants.DECREASING_ANGLE_THRESHOLD) {
          m_waitTime = Constants.CREEP_WAIT_ITERATIONS;
          m_balanceState = BalanceStates.wait;
        }

        break;

      case wait:
        // sit still for a bit
        m_driveTrainSubsystem.stop();

        // determine if we're done waiting
        if (m_waitTime <= 0) {
          // Go to balanced state, and let it sort out what happens next
          m_balanceState = BalanceStates.balanced;
        } else {
          m_waitTime--;
        }
        break;

      case idle:
        // Do nothing
        // This state should never be reached.
        // It exists to indicate that the state machine isn't running.
        break;
    }

    m_lastAngle = currentAngle;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrainSubsystem.stop();
    m_balanceState = BalanceStates.idle;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
