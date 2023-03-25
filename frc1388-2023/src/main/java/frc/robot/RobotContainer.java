// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.PrimaryArmCommand;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoKeepArmUp;
import frc.robot.commands.GrabberCommand;
import frc.robot.commands.WristCommand;
import frc.robot.commands.DriveTrainCommand.Direction;
import frc.robot.commands.DriveTrainCommand.Side;
import frc.robot.Constants.GrabberConstants;

import frc.robot.commands.DriveTrainCommand;
import frc.robot.commands.FastAutoBalance;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.LoggingSubsystem;
import frc.robot.subsystems.PrimaryArmSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.MultiChannelADIS;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.subsystems.RumbleSubsystem;
import frc.robot.subsystems.WristSubsystem;
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
  private final ArmGrabberClass m_armGrabber = new ArmGrabberClass();
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
    m_dashboard,
    m_armGrabber
  );

  private final PrimaryArmSubsystem m_primaryArmSubsystem = new PrimaryArmSubsystem(
    new WPI_TalonFX(ArmConstants.PRIMARY_ARM_CANID), 
    new DigitalInput(ArmConstants.PRIMARY_ARM_LIMIT_SWITCH_DIO_ID), 
    m_armGrabber
  );
  
  private final WristSubsystem m_wristSubsystem = new WristSubsystem(
    new WPI_TalonFX(ArmConstants.WRIST_CANID),
    new DigitalInput(ArmConstants.WRIST_LIMIT_SWITCH_DIO_ID)
  );

  //  private final GyroSubsystem m_gyroSubsystem = new GyroSubsystem(
  //  new MultiChannelADIS()
  //  );
   private final GyroSubsystem m_gyroSubsystem = new GyroSubsystem(new MultiChannelADIS(), m_dashboard);

  private final LoggingSubsystem m_LoggingSubsystem = new LoggingSubsystem();

  private final AutoMethod m_autoMethod = new AutoMethod( m_driveTrainSubsystem, m_primaryArmSubsystem, m_wristSubsystem, m_gyroSubsystem, m_dashboard );

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

    m_primaryArmSubsystem.setDefaultCommand(
      new PrimaryArmCommand(
        m_primaryArmSubsystem,
        ()-> m_opController.getRightY()
      )
    );

    m_wristSubsystem.setDefaultCommand(
      new WristCommand(
        m_wristSubsystem,
        ()-> m_opController.getLeftY()
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
    m_driverController.x().whileTrue(new AutoBalance(m_driveTrainSubsystem, m_gyroSubsystem, false));
    
    m_driverController.y().onTrue( new InstantCommand(()-> {m_gyroSubsystem.resetYAngle();} ));

    m_driverController.a().onTrue(new InstantCommand(
      ()-> {((DriveTrainCommand)m_driveTrainSubsystem.getDefaultCommand()).setDirection(Direction.reverse);}
    ));
    m_driverController.b().onTrue(new InstantCommand(
      ()-> {((DriveTrainCommand)m_driveTrainSubsystem.getDefaultCommand()).setDirection(Direction.forwards);}
    ));

    m_driverController.rightBumper().whileTrue(new FastAutoBalance(m_driveTrainSubsystem, m_gyroSubsystem));
    
    //These are the binding for the operator controller
    m_opController.leftBumper().whileTrue(new RunCommand(
      ()-> {((DriveTrainCommand)m_driveTrainSubsystem.getDefaultCommand()).turnSlow(Side.left);}, m_driveTrainSubsystem
    ));
    m_opController.rightBumper().whileTrue(new RunCommand(
      ()-> {((DriveTrainCommand)m_driveTrainSubsystem.getDefaultCommand()).turnSlow(Side.right);}, m_driveTrainSubsystem
    ));

    // m_opController.y().onTrue(new RunCommand(
    //   ()-> {m_grabberSubsystem.setGrabberPosition(5);}, m_grabberSubsystem));
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    m_gyroSubsystem.resetAllAngles();
    return m_autoMethod.getAutonomousCommand()
    .alongWith(
      new AutoKeepArmUp(m_wristSubsystem)
    ); //have to return autoMethod because it's set to m_autonomousCommand in robot class
  }

  public void setDriveTrainNeutralMode(NeutralMode mode) {
    m_driveTrainSubsystem.setNeutralMode(mode);
  }
  public double getGyroYAngle() {
    return m_gyroSubsystem.getYAngle();
  }

  public double getPrimaryArmPos() {
    return m_primaryArmSubsystem.getPrimaryArmPosition();
  }

  public void resetGrabberEncoder() {
    m_grabberSubsystem.setGrabberEncoder(0);
    m_armGrabber.hasGrabberEncoderBeenReset = true;
  }

  public void setHasGrabberBeenReset(boolean hasGrabberBeenReset) {
    m_armGrabber.hasGrabberEncoderBeenReset = hasGrabberBeenReset;
  }

  public class ArmGrabberClass {
    public double primaryArmPosition;
    public double grabberPosition;
    public boolean hasGrabberEncoderBeenReset;
  }

}