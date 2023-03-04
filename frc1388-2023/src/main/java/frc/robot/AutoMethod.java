// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoMove;
import frc.robot.commands.AutoPickUp;
import frc.robot.commands.AutoScore;
import frc.robot.commands.AutoTurn;
import frc.robot.commands.GoUntilAngle;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.GyroSubsystem;

/** Add your docs here. */
public class AutoMethod {
     
    private DriveTrainSubsystem m_driveTrainSubsystem;
    private GyroSubsystem m_gyroSubsystem;
    private final Dashboard m_Dashboard;

    public AutoMethod( DriveTrainSubsystem driveTrainSubsystem, GyroSubsystem gyroSubsystem, Dashboard Dashboard )
    {
        m_driveTrainSubsystem = driveTrainSubsystem;
        m_gyroSubsystem = gyroSubsystem;
        m_Dashboard = Dashboard;
    }

    public Command SitStillLookPretty()
    {
        return 
            new AutoMove( m_driveTrainSubsystem, 0, 0 )
        ;
    }

    public Command LeaveCommunityFar()
    {
        return 
            new AutoMove( m_driveTrainSubsystem, FieldConstants.SCORE_ZONE_TO_FAR_COMMUNITY + FieldConstants.ROBOT_LENGTH_TOTAL, 0.25)
        ;
    }

    public Command LeaveCommunityNear()
    {

        return 
            new AutoMove( m_driveTrainSubsystem, FieldConstants.SCORE_ZONE_TO_NEAR_COMMUNITY + FieldConstants.ROBOT_LENGTH_TOTAL, 0.25)
        ;
    }

    public Command Score()
    {
        return
            new AutoScore() //move arm
        ; 
    }

    public Command ScoreLeaveNear()
    {
        return 
            new AutoScore()
        .andThen(
            new AutoMove(m_driveTrainSubsystem, -(FieldConstants.SCORE_ZONE_TO_NEAR_COMMUNITY + FieldConstants.ROBOT_LENGTH_TOTAL), 0.25) //scores, backs out of community
                )
        ;
    }


    public Command ScoreLeaveFar()
    {
        return 
            new AutoScore()
        .andThen(
            new AutoMove(m_driveTrainSubsystem, -(FieldConstants.SCORE_ZONE_TO_FAR_COMMUNITY + FieldConstants.ROBOT_LENGTH_TOTAL), 0.25) //scores, backs out of community
                )
        ;
    }

    public Command ScoreLeavePickUpNear()
    {
        return 
            new AutoScore()
        .andThen(
            new AutoTurn(180, 0.5, m_gyroSubsystem, m_driveTrainSubsystem)
                )
        .andThen(
            new AutoMove(m_driveTrainSubsystem, FieldConstants.SCORE_ZONE_TO_GAME_PIECE + FieldConstants.ROBOT_LENGTH_TOTAL, 0.25)
                )
        .andThen(
            new AutoPickUp()
                )
        ;
    }

    public Command ScoreLeavePickUpFar()
    {
        return 
            new AutoScore()
        .andThen(
            new AutoTurn(180, 0.5, m_gyroSubsystem, m_driveTrainSubsystem)
                )
        .andThen(
            new AutoMove(m_driveTrainSubsystem, FieldConstants.SCORE_ZONE_TO_GAME_PIECE + FieldConstants.ROBOT_LENGTH_TOTAL, 0.25)
                )
        .andThen(
            new AutoPickUp()
                )
        ;
    }

    //Position B specific commands

    public Command ChargeStation()
    {
        return
            new AutoBalance(m_driveTrainSubsystem, m_gyroSubsystem, false) //goes until an angle. robot position is irrelevant
        ;

    }

    public Command ScoreThenCharge()
    {
        return
            new AutoScore()
        .andThen(
            new InstantCommand(()-> {m_gyroSubsystem.resetYAngle();})
            )
        .andThen(
            new AutoBalance(m_driveTrainSubsystem, m_gyroSubsystem, true)
                )
        ;
    }

    public Command OverChargeStation()
    {
        return
            new AutoMove( m_driveTrainSubsystem, FieldConstants.SCORE_ZONE_TO_FAR_COMMUNITY + FieldConstants.ROBOT_LENGTH_TOTAL, 0.25) //leaves community from mid position
        ;
    }

    public Command OverChargeAndBack()
    {
        return
            new AutoMove(m_driveTrainSubsystem, FieldConstants.SCORE_ZONE_TO_FAR_COMMUNITY + FieldConstants.ROBOT_LENGTH_TOTAL, 0.25)
        .andThen(
            new InstantCommand(()-> {m_gyroSubsystem.resetYAngle();})
                )
        .andThen(
            new AutoBalance(m_driveTrainSubsystem, m_gyroSubsystem, true) //goes up to ramp, automatically goes to autobalancing
                )
        ;
    }

    public Command ScoreOverChargeAndBack()
    {
        return 
            new AutoScore()
            .andThen(
                new AutoMove( m_driveTrainSubsystem, (FieldConstants.SCORE_ZONE_TO_CHARGE_STATION + FieldConstants.CHARGE_STATION_LENGTH), 0.25)
                )
            .andThen(
                new InstantCommand(()-> {m_gyroSubsystem.resetYAngle();})
                    )
            .andThen(
                new AutoBalance(m_driveTrainSubsystem, m_gyroSubsystem, true)
                    )
                    ; 
    }


    public Command getAutonomousCommand() {

        m_gyroSubsystem.resetYAngle();
        new InstantCommand(()-> {m_gyroSubsystem.resetZAngle();} );
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
            return new AutoMethod(m_driveTrainSubsystem, m_gyroSubsystem, m_Dashboard).SitStillLookPretty();
    
          case LEAVECOMMUNITY:
            if (position == AutoConstants.Position.FAR) {
              return new AutoMethod(m_driveTrainSubsystem, m_gyroSubsystem, m_Dashboard).LeaveCommunityFar();
            } else {// handles every position but Position C
              return new AutoMethod(m_driveTrainSubsystem, m_gyroSubsystem, m_Dashboard).LeaveCommunityNear();
            }
    
          case SCORE:
            return new AutoMethod(m_driveTrainSubsystem, m_gyroSubsystem, m_Dashboard).Score();
    
          case SCOREANDLEAVE:
            if (position == AutoConstants.Position.FAR) {
              return new AutoMethod(m_driveTrainSubsystem, m_gyroSubsystem, m_Dashboard).ScoreLeaveFar();
            } else { //handles every position but Position C
              return new AutoMethod(m_driveTrainSubsystem, m_gyroSubsystem, m_Dashboard).ScoreLeaveNear();
            }
    
          case SCORELEAVEPICKUP:
            if (position == AutoConstants.Position.FAR) {
              return new AutoMethod(m_driveTrainSubsystem, m_gyroSubsystem, m_Dashboard).ScoreLeavePickUpFar();
            } else {// handles every position but Position C
              return new AutoMethod(m_driveTrainSubsystem, m_gyroSubsystem, m_Dashboard).ScoreLeavePickUpNear();
            }
    
          case CHARGESTATION:
            return new AutoMethod(m_driveTrainSubsystem, m_gyroSubsystem, m_Dashboard).ChargeStation();
    
          case SCORETHENCHARGE:
            return new AutoMethod(m_driveTrainSubsystem, m_gyroSubsystem, m_Dashboard).ScoreThenCharge();
    
          case OVERCHARGESTATION:
            return new AutoMethod(m_driveTrainSubsystem, m_gyroSubsystem, m_Dashboard).OverChargeStation();
    
          case CHARGESTATIONBACK:
            return new AutoMethod(m_driveTrainSubsystem, m_gyroSubsystem, m_Dashboard).OverChargeAndBack();

          case SCOREOVERCHARGEBACK:
            return new AutoMethod(m_driveTrainSubsystem, m_gyroSubsystem, m_Dashboard).ScoreOverChargeAndBack();
          //case :
            //return new AutoMethod(m_driveTrainSubsystem, m_gyroSubsystem, m_Dashboard).ScoreOverChargeAndBack();
    
        }
        return null;
      }
    
}
