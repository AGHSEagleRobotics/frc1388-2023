// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoMove;
import frc.robot.commands.AutoPickUp;
import frc.robot.commands.AutoScore;
import frc.robot.commands.AutoTurn;
import frc.robot.commands.FastAutoBalance;
import frc.robot.commands.GoUntilAngle;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.GyroSubsystem;

/** Add your docs here. */
public class AutoMethod {
     
    private DriveTrainSubsystem m_driveTrainSubsystem;
    private GyroSubsystem m_gyroSubsystem;
    private final Dashboard m_Dashboard;

    //assume blue
    private double m_autoTurnAngle = AutoConstants.AUTO_TURN_ANGLE; //local variable to avoid changing constant

    public AutoMethod( DriveTrainSubsystem driveTrainSubsystem, GyroSubsystem gyroSubsystem, Dashboard Dashboard )
    {
        m_driveTrainSubsystem = driveTrainSubsystem;
        m_gyroSubsystem = gyroSubsystem;
        m_Dashboard = Dashboard;

        if( DriverStation.getAlliance() == Alliance.Red )
        {
            m_autoTurnAngle = -m_autoTurnAngle;
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
            new AutoMove(FieldConstants.SCORE_ZONE_TO_FAR_COMMUNITY + FieldConstants.ROBOT_LENGTH_TOTAL, 0.25, m_driveTrainSubsystem, m_gyroSubsystem)
        ;
    }

    public Command LeaveCommunityNear()
    {

        return 
            new AutoMove(FieldConstants.SCORE_ZONE_TO_NEAR_COMMUNITY + FieldConstants.ROBOT_LENGTH_TOTAL, 0.25, m_driveTrainSubsystem, m_gyroSubsystem)
        ;
    }

    @Deprecated
    public Command Score()
    {
        return 
        new AutoMove(18, 0.25, m_driveTrainSubsystem, m_gyroSubsystem)
        .andThen(
            new AutoMove(-18, 0.5, m_driveTrainSubsystem, m_gyroSubsystem)
                )
            // new AutoScore() //move arm
        ; 
    }

    public Command ScoreLeaveNear() //FACING FORWARDS
    { 
        return 
            new AutoMove(18, 0.25, m_driveTrainSubsystem, m_gyroSubsystem)
        .andThen(
            new AutoMove(-18, 0.5, m_driveTrainSubsystem, m_gyroSubsystem)
                )
        .andThen(
            new AutoMove( (FieldConstants.SCORE_ZONE_TO_NEAR_COMMUNITY + FieldConstants.ROBOT_LENGTH_TOTAL), 0.25, m_driveTrainSubsystem, m_gyroSubsystem) //scores, backs out of community
                )
        ;
    }


    public Command ScoreLeaveFar() //FACING FORWARDS
    {
        return 
            new AutoMove(18, 0.25, m_driveTrainSubsystem, m_gyroSubsystem)
        .andThen(
            new AutoMove(-18, 0.5, m_driveTrainSubsystem, m_gyroSubsystem)
                )
        .andThen(
            new AutoMove( (FieldConstants.SCORE_ZONE_TO_FAR_COMMUNITY + FieldConstants.ROBOT_LENGTH_TOTAL), 0.25, m_driveTrainSubsystem, m_gyroSubsystem) //scores, backs out of community
                )
        ;
    }

    public Command ScoreLeavePickUpNear()
    {
        return 
            //new AutoScore()
        //.andThen(
            new AutoTurn(180, 0.5, m_gyroSubsystem, m_driveTrainSubsystem)
        //        )
        .andThen(
            new AutoMove(FieldConstants.SCORE_ZONE_TO_GAME_PIECE + FieldConstants.ROBOT_LENGTH_TOTAL, 0.25, m_driveTrainSubsystem, m_gyroSubsystem)
                )
        //.andThen(
            //new AutoPickUp() //do we need to turn here to pick up? Will determine
               // )
        ;
    }

    public Command ScoreLeavePickUpFar()
    {
        return 
            //new AutoScore()
        //.andThen(
            new AutoTurn(180, 0.5, m_gyroSubsystem, m_driveTrainSubsystem)
        //        )
        .andThen(
            new AutoMove(FieldConstants.SCORE_ZONE_TO_GAME_PIECE + FieldConstants.ROBOT_LENGTH_TOTAL, 0.25, m_driveTrainSubsystem, m_gyroSubsystem)
                )
        //.andThen(
         //   new AutoPickUp() //do we need to turn here to pick up? Will determine
         //       )
        ;
    }

    //Position B specific commands

    public Command ChargeStation()
    {
        return 
                new InstantCommand(()-> {m_driveTrainSubsystem.resetEncoders();})
            .andThen(
                new AutoMove(42, 0.4, m_driveTrainSubsystem, m_gyroSubsystem)  
                    )
            .andThen(
                new FastAutoBalance(m_driveTrainSubsystem, m_gyroSubsystem)
                //new AutoBalance(m_driveTrainSubsystem, m_gyroSubsystem, false) //goes until an angle. robot position is irrelevant
                    )
        ;

    }

    public Command ScoreThenCharge()
    {
        return
            //new AutoScore()
            new AutoMove(18, 0.25, m_driveTrainSubsystem, m_gyroSubsystem)
            .andThen(
                new AutoMove(-18, 0.5, m_driveTrainSubsystem, m_gyroSubsystem)
                    )
            .andThen(
                new AutoMove((FieldConstants.SCORE_ZONE_TO_CHARGE_STATION-FieldConstants.ROBOT_LENGTH_TOTAL), 0.4, m_driveTrainSubsystem, m_gyroSubsystem)
                    )
            .andThen(
                new AutoBalance(m_driveTrainSubsystem, m_gyroSubsystem, false)
                    )  
            
        ;
    }

    public Command OverChargeStation()
    {
        return //start at charge station
            new AutoMove((FieldConstants.SCORE_ZONE_TO_FAR_COMMUNITY - FieldConstants.SCORE_ZONE_TO_CHARGE_STATION + FieldConstants.ROBOT_LENGTH_TOTAL + 10), 0.35, m_driveTrainSubsystem, m_gyroSubsystem) //leaves community from mid position
        ;
    }

    public Command OverChargeAndBack()
    {
        return //start at charge station
            new AutoMove((FieldConstants.CHARGE_STATION_LENGTH + FieldConstants.ROBOT_LENGTH_TOTAL + 15), 0.35, m_driveTrainSubsystem, m_gyroSubsystem)
        .andThen(
            new InstantCommand(()-> {m_driveTrainSubsystem.resetEncoders();})
                )
        .andThen(
            new AutoBalance(m_driveTrainSubsystem, m_gyroSubsystem, true)
        )
        ;
    }

    public Command ScoreOverChargeAndBack()
    {
        return 
            new AutoMove(18, 0.25, m_driveTrainSubsystem, m_gyroSubsystem)
        .andThen(
            new AutoMove(-18, 0.5, m_driveTrainSubsystem, m_gyroSubsystem)
                )
        .andThen(
            new AutoMove( (FieldConstants.SCORE_ZONE_TO_CHARGE_STATION + FieldConstants.CHARGE_STATION_LENGTH + 15), 0.35, m_driveTrainSubsystem, m_gyroSubsystem)
                )
        .andThen(
            new InstantCommand(()-> {m_gyroSubsystem.resetYAngle();})
                )
        .andThen(
            new AutoBalance(m_driveTrainSubsystem, m_gyroSubsystem, true)  
                )
                ; 
    }

    public Command hybridScoreCommand() { //FORWARD FACING, GAME PIECE ON BACK
        return new AutoMove(18, 0.25, m_driveTrainSubsystem, m_gyroSubsystem)
            .andThen(new AutoMove(-18, 0.5, m_driveTrainSubsystem, m_gyroSubsystem))
            //.andThen(new AutoMove(-(FieldConstants.SCORE_ZONE_TO_CHARGE_STATION + FieldConstants.CHARGE_STATION_LENGTH), 0.5, m_driveTrainSubsystem, m_gyroSubsystem))
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
          case HYBRIDSCORE: 
            return new AutoMethod(m_driveTrainSubsystem, m_gyroSubsystem, m_Dashboard).hybridScoreCommand();
          //case :
            //return new AutoMethod(m_driveTrainSubsystem, m_gyroSubsystem, m_Dashboard).ScoreOverChargeAndBack();
        
    
        }
        return null;
      }
    
}
