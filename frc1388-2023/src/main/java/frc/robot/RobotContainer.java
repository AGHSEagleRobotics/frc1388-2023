// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.GoUntilAngle;
import frc.robot.commands.GrabberCommand;
import frc.robot.commands.DriveTrainCommand.Direction;
import frc.robot.commands.DriveTrainCommand.Side;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.GrabberConstants;
import frc.robot.Constants.Objective;
// import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.Position;
import frc.robot.commands.AutoMove;
import frc.robot.commands.AutoPickUp;
import frc.robot.AutoMethod; //TODO review
import frc.robot.commands.AutoBalance;
import frc.robot.commands.GoUntilAngle;
import frc.robot.commands.DriveTrainCommand.Direction;
// import frc.robot.commands.Autos;
import frc.robot.commands.DriveTrainCommand;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.LoggingSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.MultiChannelADIS;

import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.subsystems.MultiChannelADIS;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.interfaces.Gyro;
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
  private final Dashboard m_Dashboard = new Dashboard();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController m_opController = new CommandXboxController(ControllerConstants.OP_CONTROLLER_PORT);

  private final DriveTrainSubsystem m_driveTrain = new DriveTrainSubsystem(
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
    new DigitalInput(ArmConstants.PRIMARY_ARM_LIMITSWITHC_DIO_ID),
    new DigitalInput(ArmConstants.PRIMARY_ARM_LIMITSWITHC_DIO_ID)
  );

  //  private final GyroSubsystem m_gyroSubsystem = new GyroSubsystem(
  //  new MultiChannelADIS()
  //  );
   private final GyroSubsystem m_gyroSubsystem = new GyroSubsystem(new MultiChannelADIS(), m_Dashboard);

  private final LoggingSubsystem m_LoggingSubsystem = new LoggingSubsystem();

  private final AutoMethod m_autoMethod = new AutoMethod( m_driveTrain, m_gyroSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_driveTrain.setDefaultCommand(
      new DriveTrainCommand(
        m_driveTrain,
        () -> m_driverController.getLeftY(),
        () -> m_driverController.getRightX(),
        () -> m_driverController.rightStick().getAsBoolean(),
        () -> m_opController.getLeftY(),
        () -> m_opController.getRightX()
      )
    );

    m_grabberSubsystem.setDefaultCommand(
      new GrabberCommand(
        m_grabberSubsystem, 
        ()-> m_opController.getLeftTriggerAxis(), 
        ()->m_opController.getRightTriggerAxis()
      )
    );

    m_ArmSubsystem.setDefaultCommand(
      new ArmCommand(m_ArmSubsystem, ()-> m_opController.getLeftY(), ()-> m_opController.getRightY())
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
      ()-> {((DriveTrainCommand)m_driveTrain.getDefaultCommand()).setDirection(Direction.reverse);}
    ));
    m_driverController.b().onTrue(new InstantCommand(
      ()-> {((DriveTrainCommand)m_driveTrain.getDefaultCommand()).setDirection(Direction.forwards);}
    ));
    
    //These are the binding for the operator controller
    m_opController.leftBumper().whileTrue(new RunCommand(
      ()-> {((DriveTrainCommand)m_driveTrain.getDefaultCommand()).turnSlow(Side.left);}, m_driveTrain
    ));
    m_opController.rightBumper().whileTrue(new RunCommand(
      ()-> {((DriveTrainCommand)m_driveTrain.getDefaultCommand()).turnSlow(Side.right);}, m_driveTrain
    ));
  }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    Objective objective = m_Dashboard.getObjective();
    Position position = m_Dashboard.getPosition();
    System.out.println(objective);

    switch ( objective ) {
      case LEAVECOMMUNITY:
      if ( position == Position.C ){
      return    
      new AutoMethod(m_driveTrain, m_gyroSubsystem).LeaveCommunityFar();
      }
      else {
      return 
      new AutoMethod(m_driveTrain, m_gyroSubsystem).LeaveCommunityNear();
      }

      case SCORE:
      return
      new AutoMethod(m_driveTrain, m_gyroSubsystem).Score();

      case SCOREANDLEAVE:
      if( position == Position.C )
      {
      return
      new AutoMethod(m_driveTrain, m_gyroSubsystem).ScoreLeaveFar();
      } 
      else{
        return 
        new AutoMethod(m_driveTrain, m_gyroSubsystem).ScoreLeaveNear();
      }

      case SCORELEAVEPICKUP:
      if ( position == Position.C )
      {
      return
      new AutoMethod(m_driveTrain, m_gyroSubsystem).ScoreLeavePickUpFar();
      }
      else
      {
        return
        new AutoMethod(m_driveTrain, m_gyroSubsystem).ScoreLeavePickUpNear();
      }

      case CHARGESTATION:
      return
      new AutoMethod(m_driveTrain, m_gyroSubsystem).ChargeStation();

      case SCORETHENCHARGE:
      return
      new AutoMethod(m_driveTrain, m_gyroSubsystem).ScoreThenCharge();

      case OVERCHARGESTATION:
      return
      new AutoMethod(m_driveTrain, m_gyroSubsystem).OverChargeStation();

      case CHARGESTATIONBACK:
      return
      new AutoMethod(m_driveTrain, m_gyroSubsystem).OverChargeAndBack();

    }
    
    return null;
  }

  public void setDriveTrainNeutralMode(NeutralMode mode) {
    m_driveTrain.setNeutralMode(mode);
  }
}