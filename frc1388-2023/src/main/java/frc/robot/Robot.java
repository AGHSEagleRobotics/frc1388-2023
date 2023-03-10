// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AutoMove;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    DataLogManager.start();
    DataLogManager.log("####### RobotInit");

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    
    CommandScheduler.getInstance().onCommandInitialize(command -> DataLogManager.log("++ " + command.getName() + " Initialized" ));
    CommandScheduler.getInstance().onCommandInterrupt(command -> DataLogManager.log("-- " + command.getName() + " Interrupted" ));
    CommandScheduler.getInstance().onCommandFinish(command -> DataLogManager.log("-- " + command.getName() + " Finished" ));
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // set motors to coast mode when user button on RoboRio is pressed
    if (RobotController.getUserButton()) {
      m_robotContainer.setDriveTrainNeutralMode(NeutralMode.Coast);
      m_robotContainer.resetGrabberEncoder();
      System.out.println("###RobotPeriodic() -> UserButtonPressed -> NeutralMode.Coast###");
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    DataLogManager.log("####### Disabled Robot");
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    DataLogManager.log("####### Autonomous Init");
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    System.out.println("setting neutral mode");
    m_robotContainer.setDriveTrainNeutralMode(NeutralMode.Brake);
    System.out.println("starting auto command");
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    // Get match info from FMS
    if (DriverStation.isFMSAttached()) {
      String fmsInfo = "FMS info: ";
      fmsInfo += " " + DriverStation.getEventName();
      fmsInfo += " " + DriverStation.getMatchType();
      fmsInfo += " match " + DriverStation.getMatchNumber();
      fmsInfo += " replay " + DriverStation.getReplayNumber();
      fmsInfo += ";  " + DriverStation.getAlliance() + " alliance";
      fmsInfo += ",  Driver Station " + DriverStation.getLocation();
      DataLogManager.log(fmsInfo);
    } else {
      DataLogManager.log("FMS not connected");

      DataLogManager.log("Match type:\t" + DriverStation.getMatchType());
      DataLogManager.log("Event name:\t" + DriverStation.getEventName());
      DataLogManager.log("Alliance:\t" + DriverStation.getAlliance());
      DataLogManager.log("Match number:\t" + DriverStation.getMatchNumber());
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    DataLogManager.log("####### Teleop Init");
    m_robotContainer.setDriveTrainNeutralMode(NeutralMode.Brake);
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("y angle > ", m_robotContainer.getGyroYAngle());
  }

  @Override
  public void testInit() {
    DataLogManager.log("####### Test Init");
    m_robotContainer.setDriveTrainNeutralMode(NeutralMode.Brake);
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    DataLogManager.log("####### Simulation Init");
    m_robotContainer.setDriveTrainNeutralMode(NeutralMode.Brake);
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
