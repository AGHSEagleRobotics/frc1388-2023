// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoMove;
import frc.robot.commands.AutoTurn;
import frc.robot.commands.AutoTurnTo;
import frc.robot.commands.GrabberCommand;
import frc.robot.commands.DriveTrainCommand.Direction;
import frc.robot.commands.DriveTrainCommand.Side;
import frc.robot.Constants.GrabberConstants;

import frc.robot.commands.DriveTrainCommand;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.LoggingSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.MultiChannelADIS;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.subsystems.RumbleSubsystem;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
  private final Dashboard m_dashboard = new Dashboard();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController m_opController = new CommandXboxController(ControllerConstants.OP_CONTROLLER_PORT);
  private final RumbleSubsystem m_rumbleSubsystem = new RumbleSubsystem(m_driverController.getHID());

  private final DriveTrainSubsystem m_driveTrainSubsystem = new DriveTrainSubsystem(
      new WPI_TalonFX(Constants.DriveTrainConstants.CANID_LEFT_FRONT),
   new WPI_TalonFX(Constants.DriveTrainConstants.CANID_LEFT_BACK), 
   new WPI_TalonFX(Constants.DriveTrainConstants.CANID_RIGHT_FRONT), 
      new WPI_TalonFX(Constants.DriveTrainConstants.CANID_RIGHT_BACK)
      );

   private final GrabberSubsystem m_grabberSubsystem = new GrabberSubsystem(
    new CANSparkMax(GrabberConstants.GRABBER_CANID, MotorType.kBrushless), 
    new DigitalInput(GrabberConstants.GRABBER_LIMIT_SWITCH_ID)
  );

  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem(
    new CANSparkMax(ArmConstants.WRIST_CANID, MotorType.kBrushless),
    new WPI_TalonFX(ArmConstants.PRIMARY_ARM_CANID),
    new DigitalInput(ArmConstants.WRIST_LIMIT_SWITCH_DIO_ID),//TODO XXX FIXME change this
    new DigitalInput(ArmConstants.PRIMARY_ARM_LIMIT_SWITCH_DIO_ID) //TODO XXX FIXME change this
  );

  //  private final GyroSubsystem m_gyroSubsystem = new GyroSubsystem(
  //  new MultiChannelADIS()
  //  );
   private final GyroSubsystem m_gyroSubsystem = new GyroSubsystem(new MultiChannelADIS(), m_dashboard);

  private final LoggingSubsystem m_LoggingSubsystem = new LoggingSubsystem();

  private final AutoMethod m_autoMethod = new AutoMethod( m_driveTrainSubsystem, m_gyroSubsystem, m_dashboard );

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // m_driveTrain.setDefaultCommand(
    //   new DriveTrainCommand(
    //     m_driveTrain,
    m_driveTrainSubsystem.setDefaultCommand(new DriveTrainCommand(
        m_driveTrainSubsystem,
        () -> m_driverController.getLeftY(),
        () -> m_driverController.getRightX(),
        () -> m_driverController.rightStick().getAsBoolean(),
        () -> m_opController.getLeftY(),
        () -> m_opController.getRightX(),
        m_rumbleSubsystem));

    m_grabberSubsystem.setDefaultCommand(
      new GrabberCommand(
        m_grabberSubsystem, 
        ()-> m_opController.getLeftTriggerAxis(), 
        ()->m_opController.getRightTriggerAxis()
      )
    );

    m_ArmSubsystem.setDefaultCommand(
      new ArmCommand(
        m_ArmSubsystem,
        ()-> m_opController.getLeftY(),
        ()-> m_opController.getRightY()
      )
    );

    // Configure the trigger bindings
    configureBindings();

    // Suppress joystick messages
    DriverStation.silenceJoystickConnectionWarning(true);
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
    //These are the binding for the driver controller
    m_driverController.y().onTrue( new InstantCommand(()-> {m_gyroSubsystem.resetYAngle();} ));

    m_driverController.a().onTrue(new InstantCommand(
      ()-> {((DriveTrainCommand)m_driveTrainSubsystem.getDefaultCommand()).setDirection(Direction.reverse);}
    ));
    m_driverController.b().onTrue(new InstantCommand(
      ()-> {((DriveTrainCommand)m_driveTrainSubsystem.getDefaultCommand()).setDirection(Direction.forwards);}
    ));
    m_driverController.rightBumper().whileTrue(new AutoBalance(m_driveTrainSubsystem, m_gyroSubsystem, false));
    
    //These are the binding for the operator controller
    m_opController.leftBumper().whileTrue(new RunCommand(
      ()-> {((DriveTrainCommand)m_driveTrainSubsystem.getDefaultCommand()).turnSlow(Side.left);}, m_driveTrainSubsystem
    ));
    m_opController.rightBumper().whileTrue(new RunCommand(
      ()-> {((DriveTrainCommand)m_driveTrainSubsystem.getDefaultCommand()).turnSlow(Side.right);}, m_driveTrainSubsystem
    ));

    m_opController.a().onTrue(new InstantCommand(
      ()-> {((ArmCommand)m_ArmSubsystem.getDefaultCommand()).toggleWristPosition();}, m_ArmSubsystem
    ));
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    m_gyroSubsystem.resetAllAngles();
//    return m_autoMethod.getAutonomousCommand(); //have to return autoMethod because it's set to m_autonomousCommand in robot class
    // return new AutoMove(48, 0.25, 0, m_driveTrainSubsystem)
    //   .andThen(new AutoTurnTo(180, 0.25, m_driveTrainSubsystem, m_gyroSubsystem))
    //   .andThen(new AutoMove(48, 0.25, 0, m_driveTrainSubsystem))
    //   .andThen(new AutoTurnTo(0, 0.25, m_driveTrainSubsystem, m_gyroSubsystem))
    //   .andThen(new AutoMove(48, 0.25, 0, m_driveTrainSubsystem))
    //   .andThen(new AutoTurnTo(180, 0.25, m_driveTrainSubsystem, m_gyroSubsystem))
    //   .andThen(new AutoMove(48, 0.25, 0, m_driveTrainSubsystem))
    //   .andThen(new AutoTurnTo(0, 0.25, m_driveTrainSubsystem, m_gyroSubsystem));
    return new AutoMove(24, 0.25, 90, m_driveTrainSubsystem, m_gyroSubsystem);
  }

  public void setDriveTrainNeutralMode(NeutralMode mode) {
    m_driveTrainSubsystem.setNeutralMode(mode);
  }

}