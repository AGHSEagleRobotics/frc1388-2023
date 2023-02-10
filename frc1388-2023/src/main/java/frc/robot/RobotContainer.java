// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.Objective;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoMove;
import frc.robot.commands.AutoPickUp;
import frc.robot.AutoMethod; //TODO review
import frc.robot.commands.Autos;
import frc.robot.commands.DriveTrainCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.GyroSubsystem;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  private final Constants m_constants = new Constants();
  private final AutoMethod m_autoMethod = new AutoMethod();


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final DriveTrain m_driveTrain = new DriveTrain
  (new WPI_TalonFX(Constants.DriveTrainConstants.CANID_LEFT_FRONT),
   new WPI_TalonFX(Constants.DriveTrainConstants.CANID_LEFT_BACK), 
   new WPI_TalonFX(Constants.DriveTrainConstants.CANID_RIGHT_FRONT), 
   new WPI_TalonFX(Constants.DriveTrainConstants.CANID_RIGHT_BACK));
  
   private final GyroSubsystem m_gyroSubsystem = new GyroSubsystem(
   new ADIS16448_IMU()
   );
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_driveTrain.setDefaultCommand(
    new DriveTrainCommand( 
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`


    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
   
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    Objective objective = m_constants.getObjective();

    switch ( objective ) {
      case LEAVECOMMUNITY:
      return    
      AutoMethod.LeaveCommunity();

      case SCORE:
      return
      AutoMethod.Score();

      case SCOREANDLEAVE:
      return
      AutoMethod.ScoreLeave();

      case SCORELEAVEPICKUP:
      return
      AutoMethod.ScoreLeavePickUp();

      case CHARGESTATION:
      return
      AutoMethod.ChargeStation();

      case OVERCHARGESTATION:
      return
      AutoMethod.OverChargeStation();

      case CHARGESTATIONBACK:
      return
      AutoMethod.OverChargeAndBack();

    }
    
    return null;
  }
}
