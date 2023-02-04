// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveTrainCommand;

import frc.robot.commands.AutoBalance;
import frc.robot.subsystems.DriveTrainSubsystem;

import frc.robot.subsystems.IMUSubsystem16448;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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


  private final DriveTrainSubsystem m_driveTrain = new DriveTrainSubsystem
  (new WPI_TalonFX(DriveTrainConstants.CANID_LEFT_FRONT),
   new WPI_TalonFX(DriveTrainConstants.CANID_LEFT_BACK), 
   new WPI_TalonFX(DriveTrainConstants.CANID_RIGHT_FRONT), 
   new WPI_TalonFX(DriveTrainConstants.CANID_RIGHT_BACK));
  

   private final IMUSubsystem16448 m_gyroSubsystem = new IMUSubsystem16448(
   new ADIS16448_IMU()
   );
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_driveTrain.setDefaultCommand(
    new DriveTrainCommand( 
    m_driveTrain,
    ()-> m_driverController.getLeftY(),
    ()-> m_driverController.getRightY(),
    ()-> m_driverController.getRightX()
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    
    //TESTING: testing out constant speed drive
    m_driverController.a().onTrue(( new InstantCommand(()-> {m_driveTrain.constantSpeedDrive(1); }) ));
    m_driverController.a().onFalse(( new InstantCommand(()-> {m_driveTrain.constantSpeedDrive(0); }) ));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
   
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    

    return new AutoBalance(m_driveTrain, m_gyroSubsystem, 0, 0);
  }
}
