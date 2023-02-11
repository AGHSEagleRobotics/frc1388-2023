// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Objective;
import frc.robot.commands.AutoMove;
import frc.robot.commands.AutoPickUp;
import frc.robot.commands.AutoScore;

/** Add your docs here. */
public class AutoMethod {
    public AutoMethod()
    {

    }

    public static Command LeaveCommunityFar()
    {
        return 
        new AutoMove( Constants.FieldConstants.SCORE_ZONE_TO_FAR_COMMUNITY + Constants.FieldConstants.ROBOT_LENGTH_BUMPERS, 0.5)
        ;
    }

    public static Command LeaveCommunityNear()
    {
        return 
        new AutoMove( Constants.FieldConstants.SCORE_ZONE_TO_NEAR_COMMUNITY + Constants.FieldConstants.ROBOT_LENGTH_BUMPERS, 0.5)
        ;
    }

    public static Command Score()
    {
        return
        new AutoScore()
        ; 
    }

    public static Command ScoreLeave()
    {
        return 
            new AutoScore()
        .andThen(
            new AutoMove(0, 0)
                )
        .andThen(
            new AutoMove(0, 0)
                )
        ;
    }

    public static Command ScoreLeavePickUp()
    {
        return 
        //new AutoScore command
        //.andThen(
        new AutoMove(0, 0)
        //         )
        .andThen(
        new AutoPickUp()
        )
        ;
    }

    //Position B specific commands

    public static Command ChargeStation()
    {
        return 
        new AutoMove(0, 0)
        //.andThen(
        // new AutoBalance()
        //        )
        ;
    }

    public static Command OverChargeStation()
    {
        return
        new AutoMove( 0, 0) //constant movement? 
        ;
    }

    public static Command OverChargeAndBack()
    {
        return
        new AutoMove(20, 0.10)
        .andThen(
        new AutoMove(-10, 0.10)
        )
        ;
    }
}
