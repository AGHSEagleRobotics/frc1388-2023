// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveTrainCommand extends CommandBase {
  // public enum DriveMode{
  //   curvature, arcade
  // }
  // private DriveMode m_driveMode = DriveMode.arcade;
  private boolean m_quickTurn;


  private final DriveTrain m_driveTrain;
  
  private Supplier<Double> m_driveLeftStickYAxis;
  private Supplier<Double> m_driveRightStickXAxis;
  private Supplier<Boolean> m_rightStickButton;

  private boolean m_lastStick = false;
  /** Creates a new DriveTrainCommand. */
  public DriveTrainCommand(
    DriveTrain driveTrain,
    Supplier<Double> driveLeftStickYAxis, 
    Supplier<Double> driveRightStickXAxis,
    Supplier<Boolean> rightStickButton
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);

    m_driveTrain = driveTrain;
    m_driveLeftStickYAxis = driveLeftStickYAxis;
    m_driveRightStickXAxis = driveRightStickXAxis;
    m_rightStickButton = rightStickButton;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = -m_driveLeftStickYAxis.get();
    double  rotation = -m_driveRightStickXAxis.get();
    speed = MathUtil.applyDeadband(speed, .1);
    rotation = MathUtil.applyDeadband(rotation, .1);
    if (m_rightStickButton.get() && !m_lastStick) {
      m_quickTurn = !m_quickTurn;
    }
    m_driveTrain.curvatureDrive(speed, rotation, m_quickTurn);
    m_lastStick = m_rightStickButton.get();
    System.out.println("speed: " + speed + " rotation: " + rotation);
  }

    // ??????????????????????????????????????????????????
    // System.out.println("mode: " + m_driveMode.name() + " speed: " + speed + " rotation: " + rotation);
    // if (Math.random() > 0.7) System.out.println("current: " + m_rightStickButton.get() + "\t\tlast: " + m_lastStick);
    // switch (m_driveMode) {
    //   // case arcade: m_driveTrain.arcadeDrive(speed, rotation);
    //   //   break;
    //   // case curvature: m_driveTrain.curvatureDrive(speed, rotation, true);  
    //   //   break;
    //   // default: m_driveTrain.arcadeDrive(0, 0);
    // }
    // ??????????????????????????????????????????????????
          // switch (m_driveMode) {
        //   case arcade: m_driveMode = DriveMode.curvature;
        //     break;
        //   case curvature: m_driveMode = DriveMode.arcade;
        //     break;
    //???????????????????????????????????????????



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
