// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.GoUntilAngle;
import frc.robot.commands.DriveTrainCommand.Direction;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveTrainCommand;
import frc.robot.commands.AutoBalance;

import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.DriveTrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.subsystems.MultiChannelADIS;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final Dashboard m_Dashboard = new Dashboard();


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);


  private final DriveTrain m_driveTrain = new DriveTrain
  (new WPI_TalonFX(Constants.DriveTrainConstants.CANID_LEFT_FRONT),
   new WPI_TalonFX(Constants.DriveTrainConstants.CANID_LEFT_BACK), 
   new WPI_TalonFX(Constants.DriveTrainConstants.CANID_RIGHT_FRONT), 
   new WPI_TalonFX(Constants.DriveTrainConstants.CANID_RIGHT_BACK));
  

  //  private final GyroSubsystem m_gyroSubsystem = new GyroSubsystem(
  //  new MultiChannelADIS()
  //  );

   final DriveTrainCommand m_driveCommand = new DriveTrainCommand( 
    m_driveTrain,
    ()-> m_driverController.getLeftY(),
    ()-> m_driverController.getRightX(),
    ()-> m_driverController.rightStick().getAsBoolean()
    // ()-> m_driverController.a().getAsBoolean(),
    // ()-> m_driverController.b().getAsBoolean(),
    // m_driverController
  );
   private final GyroSubsystem m_gyroSubsystem = new GyroSubsystem(new MultiChannelADIS(), m_Dashboard);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {


      
    m_driveTrain.setDefaultCommand( new DriveTrainCommand( 
      m_driveTrain,
      ()-> m_driverController.getLeftY(),
      ()-> m_driverController.getRightX(),
      ()-> m_driverController.rightStick().getAsBoolean()
    ));

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
   
    m_driverController.y().onTrue( new InstantCommand(()-> {m_gyroSubsystem.resetYAngle();} ));

    // TESTING: testing out constant speed drive
    m_driverController.a().whileTrue( new RepeatCommand(new InstantCommand(()-> {m_driveTrain.constantSpeedDrive(12); }) ));
    m_driverController.a().onFalse( new InstantCommand(()-> {m_driveTrain.constantSpeedDrive(0); }) );    
    m_driverController.a().onTrue(new RepeatCommand(new InstantCommand(
      ()-> {m_driveCommand.setForwards(Direction.reverse);}
    )));
  
    m_driverController.b().onTrue(new RepeatCommand(new InstantCommand(
      ()-> {m_driveCommand.setForwards(Direction.forwards);}
    )));
  }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    
    System.out.println("Get Auto Command");
    return new GoUntilAngle(m_driveTrain, m_gyroSubsystem, 5)
      .andThen(new AutoBalance(m_driveTrain, m_gyroSubsystem));
  }

  public void setNeutralMode(NeutralMode mode) {
    m_driveTrain.setNeutralMode(mode);
  }
}