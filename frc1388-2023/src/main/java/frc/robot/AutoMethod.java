// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.Objective;
import frc.robot.commands.AutoMove;
import frc.robot.commands.AutoPickUp;
import frc.robot.commands.AutoScore;
import frc.robot.commands.AutoTurn;

/** Add your docs here. */
public class AutoMethod {
    public AutoMethod()
    {

    }

    public static Command LeaveCommunityFar()
    {
        return 
        new AutoMove( FieldConstants.SCORE_ZONE_TO_FAR_COMMUNITY + FieldConstants.ROBOT_LENGTH_BUMPERS, 0.5)
        ;
    }

    public static Command LeaveCommunityNear()
    {
        return 
        new AutoMove( FieldConstants.SCORE_ZONE_TO_NEAR_COMMUNITY + FieldConstants.ROBOT_LENGTH_BUMPERS, 0.5)
        ;
    }

    public static Command Score()
    {
        return
        new AutoScore() //move arm
        ; 
    }

    public static Command ScoreLeaveNear()
    {
        return 
            new AutoScore()
        .andThen(
            new AutoMove(-(FieldConstants.SCORE_ZONE_TO_NEAR_COMMUNITY + FieldConstants.ROBOT_LENGTH_BUMPERS), 0.5)
                )
        ;
    }


    public static Command ScoreLeaveFar()
    {
        return 
            new AutoScore()
        .andThen(
            new AutoMove(-(FieldConstants.SCORE_ZONE_TO_FAR_COMMUNITY + FieldConstants.ROBOT_LENGTH_BUMPERS), 0.5)
                )
        ;
    }

    public static Command ScoreLeavePickUpNear()
    {
        return 
            new AutoScore()
        .andThen(
            new AutoTurn(0.5, 180)
        )
        .andThen(
            new AutoMove(FieldConstants.SCORE_ZONE_TO_GAME_PIECE + FieldConstants.ROBOT_LENGTH_BUMPERS, 0.5)
                )
        .andThen(
            new AutoPickUp()
        )
        ;
    }

    public static Command ScoreLeavePickUpFar()
    {
        return 
            new AutoScore()
        .andThen(
            new AutoTurn(0.5, 180)
        )
        .andThen(
            new AutoMove(FieldConstants.SCORE_ZONE_TO_GAME_PIECE + FieldConstants.ROBOT_LENGTH_BUMPERS, 0.5)
                )
        .andThen(
            new AutoPickUp()
        )
        ;
    }

    //Position B specific commands

    public static Command ChargeStation()
    {
        return 
        new AutoMove(40, 0.5)
        //.andThen(
        // new AutoBalance()
        //        )
        ;
    }

    public static Command ScoreThenCharge()
    {
        return
        new AutoScore()
        .andThen(
            new AutoTurn(0.5, 180)
        )
        .andThen(
            new AutoMove(40, 0.5)
        )
        //.andThen(
        // new AutoBalance()
        //        )
        ;
    }

    public static Command OverChargeStation()
    {
        return
        new AutoMove( 190 + FieldConstants.ROBOT_LENGTH_BUMPERS, 0.5) // 190 is a guess to end of charge station 
        ;
    }

    public static Command OverChargeAndBack()
    {
        return
        new AutoMove(190, 0.5)
        //.andThen(
        //new AutoBalance()
        //        )
        ;
    }
}
