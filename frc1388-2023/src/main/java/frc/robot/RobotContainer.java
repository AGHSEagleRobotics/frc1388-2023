// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.Objective;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.Position;
import frc.robot.commands.AutoMove;
import frc.robot.commands.AutoPickUp;
import frc.robot.AutoMethod; //TODO review
import frc.robot.commands.AutoBalance;
import frc.robot.commands.DriveTrainCommand;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.MultiChannelADIS;

import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
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
  
  
  private final DriveTrain m_driveTrain = new DriveTrain
  (new WPI_TalonFX(Constants.DriveTrainConstants.CANID_LEFT_FRONT),
  new WPI_TalonFX(Constants.DriveTrainConstants.CANID_LEFT_BACK), 
  new WPI_TalonFX(Constants.DriveTrainConstants.CANID_RIGHT_FRONT), 
  new WPI_TalonFX(Constants.DriveTrainConstants.CANID_RIGHT_BACK));
  
  
  private final GyroSubsystem m_gyroSubsystem = new GyroSubsystem(
    new MultiChannelADIS()
    );

  private final AutoMethod m_autoMethod = new AutoMethod( m_driveTrain, m_gyroSubsystem );

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

    // TESTING: testing out constant speed drive
    m_driverController.a().whileTrue( new RepeatCommand(new InstantCommand(()-> {m_driveTrain.constantSpeedDrive(12); }) ));
    m_driverController.a().onFalse( new InstantCommand(()-> {m_driveTrain.constantSpeedDrive(0); }) );
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
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

  public void setNeutralMode(NeutralMode mode) {
    m_driveTrain.setNeutralMode(mode);
}
}
