// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoBalanceConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.GyroSubsystem;

public class AutoBalance extends CommandBase {

  private enum BalanceStates {
    approachingRamp,
    driveOnRamp,
    moveToBalance,
    balanced
  }
  private BalanceStates m_balanceState = BalanceStates.approachingRamp;

  private DriveTrainSubsystem m_driveTrainSubsystem;
  private GyroSubsystem m_gyroSubsystem;

  private double m_angle1 = 0;
  private double m_angle2 = 0;
  private double m_angle3 = 0;
  private int m_tickCounter = 0;
  private int m_outOfBalanceCounter = 0;

  /** Creates a new AutoTurn. */
  public AutoBalance(DriveTrainSubsystem driveTrainSubsystem, GyroSubsystem gyroSubsystem) {
    m_driveTrainSubsystem = driveTrainSubsystem;
    m_gyroSubsystem = gyroSubsystem;

    addRequirements(driveTrainSubsystem);   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_tickCounter++;

    double pSpeed = 0;
    double currentAngle = m_gyroSubsystem.getYAngle();
    double averageAngle = Math.abs((m_angle1 + m_angle2 + m_angle3) / 3);

    // if( averageAngle - Math.abs(currentAngle) > 1){
    //   if( Math.abs(currentAngle) < 13.5 ){
    //     pSpeed = 0;
    //   }
    // } else {
    //       //double pSpeed = angle * DriveTrainConstants.0.02;
    //   pSpeed = Math.pow(((Math.abs(currentAngle))/15), 3);
    //   // System.out.println("(abs(angle)/15)^2.5 :  " + pSpeed);
    //   pSpeed = Math.copySign(pSpeed, currentAngle);
    //   // System.out.println("pSpeed copySign :  " + pSpeed);
    //   pSpeed = pSpeed * -AutoBalanceConstants.HIGH_SPEED;
    //   pSpeed = MathUtil.clamp(pSpeed, -AutoBalanceConstants.HIGH_SPEED, AutoBalanceConstants.HIGH_SPEED);
    //   // System.out.println("pSpeed :  " + pSpeed);
    //   m_driveTrainSubsystem.constantSpeedDrive(pSpeed);
    //   // m_driveTrainSubsystem.constantSpeedDrive(Math.copySign(8, -currentAngle));
    
    // }
    // System.out.println("Auto balance current speed: " + pSpeed);
    // m_driveTrainSubsystem.constantSpeedDrive(pSpeed);
    constantSpeedBalance(0.5);
    
    // switch (m_balanceState) {
    //   case approachingRamp:{
    //     if(Math.abs(m_gyroSubsystem.getYAngle()) > AutoBalanceConstants.CHARGE_STATION_DETECTION_ANGLE) {
    //       m_balanceState = BalanceStates.driveOnRamp;
    //     }
    //     m_driveTrainSubsystem.constantSpeedDrive(AutoBalanceConstants.GO_UNTIL_ANGLE);
    //     break;
    //   }
      
    //   case driveOnRamp:{
    //     m_balanceState = BalanceStates.goingUpRamp;
    //     break;
    //   }
        
        
  
    //   case goingUpRamp: {
    //     if (m_gyroSubsystem.getYAngle() + 1 < averageAngle) {
    //       m_balanceState = BalanceStates.rampTiltingDown;
    //     }
    //     constantSpeedBalance(AutoBalanceConstants.HIGH_SPEED);
    //     break;
    //   }
  
    //   case rampTiltingDown:{
    //     if(m_gyroSubsystem.getYAngle() < -2.5) {
    //       m_balanceState = BalanceStates.overshooting;
    //     }
    //     if(m_gyroSubsystem.getYAngle() > 2.5) {
    //       m_balanceState = BalanceStates.goingUpRamp;
    //     }
    //     m_driveTrainSubsystem.arcadeDrive(0, 0);
    //     break;
    //   }
  
    //   case overshooting:{
    //     if(Math.abs(m_gyroSubsystem.getYAngle()) > 2.5) {
    //       m_balanceState = BalanceStates.rampTiltingDown;
    //     }
    //     constantSpeedBalance(AutoBalanceConstants.LOW_SPEED);
    //   } 
    //  }
    SmartDashboard.putString("balanceState", m_balanceState.toString());
     
    switch (m_balanceState) {
      case approachingRamp:
        m_driveTrainSubsystem.constantSpeedDrive(AutoBalanceConstants.GO_UNTIL_ANGLE_SPEED);

        if (Math.abs(currentAngle) >= AutoBalanceConstants.CHARGE_STATION_DETECTION_ANGLE) {
          m_balanceState = BalanceStates.moveToBalance;
        }

        if (Math.abs(currentAngle) > AutoBalanceConstants.CHARGE_STATION_DETECTION_ANGLE) {
          // reset encoder before switching to driveOnRamp
          m_driveTrainSubsystem.resetLeftEncoder();
          m_balanceState = BalanceStates.driveOnRamp;
        }
        break;
      
      case driveOnRamp:
        // Drive a number of inches forward then move to balance state
        m_driveTrainSubsystem.constantSpeedDrive(AutoBalanceConstants.DRIVE_ON_RAMP_SPEED);

        // if a distance is reached (number of inches)
        if (Math.abs(m_driveTrainSubsystem.getLeftEncoderDistance()) > AutoBalanceConstants.DRIVE_ON_RAMP_DISTANCE) {
          m_balanceState = BalanceStates.moveToBalance;
        }
        break;
        
      case moveToBalance:
        constantSpeedBalance(AutoBalanceConstants.HIGH_SPEED);

        if (averageAngle - Math.abs(currentAngle) > 1) {
          m_balanceState = BalanceStates.balanced;
        }
        break;
  
      case balanced:
        m_driveTrainSubsystem.arcadeDrive(0, 0);
        
        if (Math.abs(currentAngle) <= 2.5){
          m_outOfBalanceCounter = 0;
        }
        else { // angle >= 2.5
          m_outOfBalanceCounter++;
        }
        if (m_outOfBalanceCounter >= AutoBalanceConstants.NOT_BALANCED_TICKS) {
          m_balanceState = BalanceStates.moveToBalance;
        }
        break;
    }
    constantSpeedBalance(AutoBalanceConstants.HIGH_SPEED);
    
    if( m_tickCounter == 1){ m_angle1 = currentAngle; }
    if( m_tickCounter == 2){ m_angle2 = currentAngle; }
    if( m_tickCounter == 3){ m_angle3 = currentAngle; m_tickCounter=0; }

    // averageAngleList.remove(0);
    // averageAngleList.add(currentAngle);

    // System.out.println("Angle: "+angle + "/t Speed: " + pSpeed);
    // SmartDashboard.putNumber("TargetSpeed", pSpeed);
    SmartDashboard.putNumber("angle", currentAngle);
  }


  public void constantSpeedBalance(double maxSpeed) {
    double pSpeed;
    double currentAngle = m_gyroSubsystem.getYAngle();

    pSpeed = Math.pow(((Math.abs(currentAngle))/8), 3);
    pSpeed = Math.copySign(pSpeed, currentAngle);
    pSpeed = pSpeed * -AutoBalanceConstants.HIGH_SPEED;
    pSpeed = MathUtil.clamp(pSpeed, -AutoBalanceConstants.HIGH_SPEED, AutoBalanceConstants.HIGH_SPEED);
    m_driveTrainSubsystem.constantSpeedDrive(pSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrainSubsystem.curvatureDrive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
