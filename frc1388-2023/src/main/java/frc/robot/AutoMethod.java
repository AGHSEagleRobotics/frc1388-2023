// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoMove;
import frc.robot.commands.AutoMovePrimaryArm;
import frc.robot.commands.AutoMoveWrist;
import frc.robot.commands.AutoPickUp;
import frc.robot.commands.AutoScore;
import frc.robot.commands.AutoSetGrabberPosition;
import frc.robot.commands.AutoTurn;
import frc.robot.commands.AutoWristSetPoint;
import frc.robot.commands.FastAutoBalance;
import frc.robot.commands.GoUntilAngle;
import frc.robot.commands.AutoWristSetPoint.WristPositions;
import frc.robot.subsystems.PrimaryArmSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.GyroSubsystem;

/** Add your docs here. */
//All robot positions are facing forward
public class AutoMethod {
     
    private DriveTrainSubsystem m_driveTrainSubsystem;
    private GyroSubsystem m_gyroSubsystem;
    private final Dashboard m_Dashboard;
    private final PrimaryArmSubsystem m_primaryArmSubsystem;
    private final WristSubsystem m_wristSubsystem;
    private final GrabberSubsystem m_grabberSubsystem;

    //assume blue
    private double m_autoTurnAngle = AutoConstants.AUTO_TURN_ANGLE; //local variable to avoid changing constant

    public AutoMethod(DriveTrainSubsystem driveTrainSubsystem, PrimaryArmSubsystem primaryArmSubsystem, WristSubsystem wristSubsystem, GrabberSubsystem grabberSubsystem, GyroSubsystem gyroSubsystem, Dashboard Dashboard)
    {
        m_driveTrainSubsystem = driveTrainSubsystem;
        m_primaryArmSubsystem = primaryArmSubsystem;
        m_wristSubsystem = wristSubsystem;
        m_grabberSubsystem = grabberSubsystem;
        m_gyroSubsystem = gyroSubsystem;
        m_Dashboard = Dashboard;

        if( DriverStation.getAlliance() == Alliance.Red )
        {
            m_autoTurnAngle = -m_autoTurnAngle; //TODO remove is drive team does not want. Makes turns mirrored
        }
    }

    public Command SitStillLookPretty()
    {
        return 
            new AutoMove( 0, 0, m_driveTrainSubsystem, m_gyroSubsystem )
        ;
    }

    public Command LeaveCommunityFar()
    {
        return 
            new AutoMove(-(FieldConstants.SCORE_ZONE_TO_FAR_COMMUNITY + FieldConstants.ROBOT_LENGTH_TOTAL), 0.25, m_driveTrainSubsystem, m_gyroSubsystem)
        ;
    }

    public Command LeaveCommunityNear()
    {

        return 
            new AutoMove(-(FieldConstants.SCORE_ZONE_TO_NEAR_COMMUNITY + FieldConstants.ROBOT_LENGTH_TOTAL), 0.25, m_driveTrainSubsystem, m_gyroSubsystem)
        ;
    }

    public Command Score()
    {
        return 
            new AutoMovePrimaryArm(m_primaryArmSubsystem, 0.22)
        .alongWith(
            new WaitCommand(1.0)
        .andThen(
            new AutoWristSetPoint(m_wristSubsystem, WristPositions.extend)
                )
        )
        .andThen(
            new AutoWristSetPoint(m_wristSubsystem, WristPositions.retract)
                .withTimeout(2)
            .alongWith(
                new WaitCommand(0.3)
            .andThen(
                new AutoMovePrimaryArm(m_primaryArmSubsystem, 0.0)
                    )
            )
        )
        ; 
    }

    public Command ScoreLeaveNear()
    { 
        return 
            Score()
        .andThen(
            LeaveCommunityNear()
        )
        ;
    }

    public Command ScoreLeaveFar()
    {
        return Score()
                .andThen(
                        LeaveCommunityFar());
    }

    public Command ScoreLeavePickUpNear() {
        return Score()
                .andThen(
                        LeaveCommunityNear())
                .andThen(
                        new AutoTurn(180, 0.25, m_gyroSubsystem, m_driveTrainSubsystem)
                        )
                        // new AutoPickUp()
                        ;
    }

    public Command ScoreLeavePickUpFar()
    {
        return 
            Score()
        .andThen(
            LeaveCommunityFar()
               )
        .andThen(
            new AutoTurn(180, 0.25, m_gyroSubsystem, m_driveTrainSubsystem)
                )
        ;
    }

    //Position B specific commands

    public Command ChargeStation()
    {
        return 
            new AutoBalance(m_driveTrainSubsystem, m_gyroSubsystem, true)
            //just in case fastBalance code below
            //new AutoMove(42, 0.4, m_driveTrainSubsystem, m_gyroSubsystem)  
                //)
        // .andThen(
            //new FastAutoBalance(m_driveTrainSubsystem, m_gyroSubsystem)
        ;

    }

    public Command ScoreThenCharge() 
    {
        return
            Score()
        .andThen(
            new AutoBalance(m_driveTrainSubsystem, m_gyroSubsystem, true)
                )  
            
        ;
    }

    public Command OverChargeStation()
    {
        return //start at charge station
            new AutoMove(-(FieldConstants.SCORE_ZONE_TO_FAR_COMMUNITY - FieldConstants.SCORE_ZONE_TO_CHARGE_STATION + FieldConstants.ROBOT_LENGTH_TOTAL + 10), 0.35, m_driveTrainSubsystem, m_gyroSubsystem) //leaves community from mid position
        ;
    }

    public Command OverChargeAndBack()
    {
        return //start at charge station
            new AutoMove(-(FieldConstants.CHARGE_STATION_LENGTH + FieldConstants.ROBOT_LENGTH_TOTAL + AutoConstants.CHARGE_STATION_MAGIC_NUM), 0.35, m_driveTrainSubsystem, m_gyroSubsystem)
        .andThen(
            new AutoBalance(m_driveTrainSubsystem, m_gyroSubsystem, false)
                )
        ;
    }

    public Command ScoreOverChargeAndBack()
    {
        return 
            Score()
        .andThen(
            new AutoMove( -(FieldConstants.SCORE_ZONE_TO_CHARGE_STATION + FieldConstants.CHARGE_STATION_LENGTH + AutoConstants.CHARGE_STATION_MAGIC_NUM), 0.35, m_driveTrainSubsystem, m_gyroSubsystem)
                )
        .andThen(
            new AutoBalance(m_driveTrainSubsystem, m_gyroSubsystem, false)  
                )
                ; 
    }

    public Command hybridScoreCommand() { //NON-SCORE-ZONE FACING, GAME PIECE ON BACK
        return new AutoMove(18, 0.25, m_driveTrainSubsystem, m_gyroSubsystem)
            .andThen(new AutoMove(-18, 0.5, m_driveTrainSubsystem, m_gyroSubsystem))
            ;
    }


    public Command getAutonomousCommand() {

        m_gyroSubsystem.resetYAngle();
        m_gyroSubsystem.resetZAngle();
        AutoConstants.Objective objective = m_Dashboard.getObjective();
        AutoConstants.Position position = m_Dashboard.getPosition();
        DataLogManager.log("####### objective:" + objective);
        DataLogManager.log("####### position:" + position);
    
        if (objective == null || position == null) {
          return null;
        }
    
        switch (objective) {
    
          case SITSTILL:
            return SitStillLookPretty();
    
          case LEAVECOMMUNITY:
            if (position == AutoConstants.Position.FAR) {
              return LeaveCommunityFar();
            } else {// handles every position but Position C
              return LeaveCommunityNear();
            }
    
          case SCORE:
            return Score();
    
          case SCOREANDLEAVE:
            if (position == AutoConstants.Position.FAR) {
              return ScoreLeaveFar();
            } else { //handles every position but Position C
              return ScoreLeaveNear();
            }
    
          case SCORELEAVEPICKUP:
            if (position == AutoConstants.Position.FAR) {
              return ScoreLeavePickUpFar();
            } else {// handles every position but Position C
              return ScoreLeavePickUpNear();
            }
    
          case CHARGESTATION:
            return ChargeStation();
    
          case SCORETHENCHARGE:
            return ScoreThenCharge();
    
          case OVERCHARGESTATION:
            return OverChargeStation();
    
          case CHARGESTATIONBACK:
            return OverChargeAndBack();

          case SCOREOVERCHARGEBACK:
            return ScoreOverChargeAndBack();

          case HYBRIDSCORE: 
            return hybridScoreCommand();
          //case :
            //return new AutoMethod(m_driveTrainSubsystem, m_gyroSubsystem, m_Dashboard).ScoreOverChargeAndBack();
        
    
        }
        return null;
      }
    
}
