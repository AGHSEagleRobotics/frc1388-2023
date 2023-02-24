// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoBalanceConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoMove;
import frc.robot.commands.AutoPickUp;
import frc.robot.commands.AutoScore;
import frc.robot.commands.AutoTurn;
import frc.robot.commands.GoUntilAngle;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.MultiChannelADIS;

/** Add your docs here. */
public class AutoMethod {
     
    private DriveTrainSubsystem m_driveTrain;
    private GyroSubsystem m_gyroSubsystem;
    public AutoMethod( DriveTrainSubsystem driveTrain, GyroSubsystem gyroSubsystem )
    {
        m_driveTrain = driveTrain;
        m_gyroSubsystem = gyroSubsystem;
    }

    public Command LeaveCommunityFar()
    {
        return 
            new AutoMove( FieldConstants.SCORE_ZONE_TO_FAR_COMMUNITY + FieldConstants.ROBOT_LENGTH_BUMPERS, 0.5)
        ;
    }

    public Command LeaveCommunityNear()
    {
        return 
            new AutoMove( FieldConstants.SCORE_ZONE_TO_NEAR_COMMUNITY + FieldConstants.ROBOT_LENGTH_BUMPERS, 0.5)
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
            new AutoMove(-(FieldConstants.SCORE_ZONE_TO_NEAR_COMMUNITY + FieldConstants.ROBOT_LENGTH_BUMPERS), 0.5)
        )
        ;
    }


    public Command ScoreLeaveFar()
    {
        return 
            new AutoScore()
        .andThen(
            new AutoMove(-(FieldConstants.SCORE_ZONE_TO_FAR_COMMUNITY + FieldConstants.ROBOT_LENGTH_BUMPERS), 0.5)
        )
        ;
    }

    public Command ScoreLeavePickUpNear()
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

    public Command ScoreLeavePickUpFar()
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

    public Command ChargeStation()
    {
        // return 
        //     new AutoMove(40, 0.5)
        // .andThen(
        //     new AutoBalance(m_driveTrain, m_gyroSubsystem)
        //     )
        // ; 

        return 
            new GoUntilAngle(m_driveTrain, m_gyroSubsystem, 14)
        .andThen(
            new AutoBalance(m_driveTrain, m_gyroSubsystem)
        )
        ;

    }

    public Command ScoreThenCharge()
    {
        return
        new AutoScore()
        .andThen(
            new AutoTurn(0.5, 180)
        )
        .andThen(
            new AutoMove(40, 0.5)
        )
        .andThen(
            new AutoBalance(m_driveTrain, m_gyroSubsystem)
        )
        ;
    }

    public Command OverChargeStation()
    {
        return
            new AutoMove( 190 + FieldConstants.ROBOT_LENGTH_BUMPERS, 0.5) // 190 is a guess to end of charge station 
        ;
    }

    public Command OverChargeAndBack()
    {
        return
            new AutoMove(190, 0.5)
        .andThen(
            new AutoBalance(m_driveTrain, m_gyroSubsystem)
        )
        ;
    }
}
