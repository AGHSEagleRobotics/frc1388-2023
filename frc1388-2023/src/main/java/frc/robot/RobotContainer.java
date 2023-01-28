// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveTrainCommand;
import frc.robot.subsystems.DriveTrain;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final Dashboard m_Dashboard = new Dashboard();
  private final PneumaticHub m_PneumaticHub = new PneumaticHub(Constants.Pneumatics.kCANIDPneumaticHub);
  
  public final DoubleSolenoid m_DoubleSolenoid = new DoubleSolenoid(
    Constants.Pneumatics.kCANIDPneumaticHub, PneumaticsModuleType.REVPH, 
    Constants.Pneumatics.kChExtend, Constants.Pneumatics.kChRetract);
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandXboxController m_driverController =
  //     new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final XboxController m_driverController = new XboxController(0);
  private final CommandXboxController m_OpController = new CommandXboxController(1);
  private final DriveTrain m_driveTrain = new DriveTrain
  (new WPI_TalonFX(0),
   new WPI_TalonFX(1), 
   new WPI_TalonFX(2), 
   new WPI_TalonFX(3));
  
   //private final Air m_air = new Air();
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
    
    // m_driverController.getAButto
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // new JoystickButton(m_OpController, XboxController.Button.kA.value).onTrue(
    //   new SolenoidMove(m_DoubleSolenoid));

    //Old version without CommonOpController
    // new JoystickButton(m_OpController, XboxController.Button.kA.value)
    //   .onTrue( new InstantCommand(()-> { m_DoubleSolenoid.set(Value.kForward); }) );

    m_OpController.a().onTrue(( new InstantCommand(()-> { m_DoubleSolenoid.set(Value.kForward); }) ));
    m_OpController.b().onTrue(( new InstantCommand(()-> { m_DoubleSolenoid.set(Value.kReverse); }) ));
    //new InstantCommand needed to use lambda - takes runnable

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    
    return null;
  }
}
