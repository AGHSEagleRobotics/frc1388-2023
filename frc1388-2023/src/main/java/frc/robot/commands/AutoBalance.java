// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoBalanceConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.GyroSubsystem;

public class AutoBalance extends CommandBase {

  private enum BalanceStates {
    approachingRamp,
    goingUpRamp,
    rampTiltingDown,
    overshooting
  }
  private BalanceStates m_balanceState = BalanceStates.approachingRamp;

  private DriveTrain m_driveTrainSubsystem;
  private GyroSubsystem m_gyroSubsystem;

  private double m_angle1 = 0;
  private double m_angle2 = 0;
  private double m_angle3 = 0;
  private int m_tickCounter = 0;

  private final PIDController m_pidController = new PIDController(0.02, 0, 0);

  private ArrayList<Double> averageAngleList = new ArrayList<Double>();

  /** Creates a new AutoTurn. */
  public AutoBalance(DriveTrain driveTrainSubsystem, GyroSubsystem gyroSubsystem) {
    m_driveTrainSubsystem = driveTrainSubsystem;
    m_gyroSubsystem = gyroSubsystem;

    averageAngleList.add(0.0);
    averageAngleList.add(0.0);
    averageAngleList.add(0.0);
    addRequirements(driveTrainSubsystem);

    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // m_driveTrainSubsystem.resetGyro();
   // m_driveTrainSubsystem.setDeadbandZero();
   System.out.println("#########Stargin auto balacne #############");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //  double turnSpeed;
    m_tickCounter++;

    double pSpeed;
    double currentAngle = m_gyroSubsystem.getYAngle();
    double averageAngle = Math.abs((m_angle1 + m_angle2 + m_angle3) / 3);
    // double averageAngle = Math.abs((averageAngleList.get(0) + averageAngleList.get(1) + averageAngleList.get(2)) /3.0);
    // System.out.println("Angle1: "+ m_angle1 + "   Angle2: " + m_angle2 + "   Angle3: " + m_angle3);
    // System.out.println("Average Angle: "+ averageAngle);
  
    System.out.println("Auto balance current angle: " + currentAngle);

    if( averageAngle - Math.abs(currentAngle) > 1){
    //if( Math.abs(angle) < 13.5 ){
      pSpeed = 0;
    }
    else {
          //double pSpeed = angle * DriveTrainConstants.0.02;
      pSpeed = Math.pow(((Math.abs(currentAngle))/15), 3);
      // System.out.println("(abs(angle)/15)^2.5 :  " + pSpeed);
      pSpeed = Math.copySign(pSpeed, currentAngle);
      // System.out.println("pSpeed copySign :  " + pSpeed);
      pSpeed = pSpeed * -AutoBalanceConstants.HIGH_SPEED;
      pSpeed = MathUtil.clamp(pSpeed, -AutoBalanceConstants.HIGH_SPEED, AutoBalanceConstants.HIGH_SPEED);
      // System.out.println("pSpeed :  " + pSpeed);
      m_driveTrainSubsystem.constantSpeedDrive(pSpeed);
      // m_driveTrainSubsystem.constantSpeedDrive(Math.copySign(8, -currentAngle));
    
    }
    System.out.println("Auto balance current speed: " + pSpeed);
    
    
    switch (m_balanceState) {
      case approachingRamp:
        if(Math.abs(m_gyroSubsystem.getYAngle()) > AutoBalanceConstants.CHARGE_STATION_DETECTION_ANGLE) {
          m_balanceState = BalanceStates.goingUpRamp;
        }
        m_driveTrainSubsystem.arcadeDrive(0.5, 0);
        break;
  
      case goingUpRamp: 
        if (m_gyroSubsystem.getYAngle() + 1 < averageAngle) {
          m_balanceState = BalanceStates.rampTiltingDown;
        }
        constantSpeedBalance(AutoBalanceConstants.HIGH_SPEED);
        break;
  
      case rampTiltingDown:
        if(m_gyroSubsystem.getYAngle() < -2.5) {
          m_balanceState = BalanceStates.overshooting;
        }
        if(m_gyroSubsystem.getYAngle() > 2.5) {
          m_balanceState = BalanceStates.goingUpRamp;
        }
        m_driveTrainSubsystem.arcadeDrive(0, 0);
        break;
  
      case overshooting: if(Math.abs(m_gyroSubsystem.getYAngle()) > 2.5) {
        m_balanceState = BalanceStates.rampTiltingDown;
      }
      constantSpeedBalance(AutoBalanceConstants.LOW_SPEED);
     }
    




    /* TODO test constant code
    double constantSpeed = DriveTrainConstants.BALANCE_CONSTANT;
    m_driveTrainSubsystem.tankDrive(constantSpeed, constantSpeed);
    */
    if( m_tickCounter == 1){ m_angle1 = currentAngle; }
    if( m_tickCounter == 2){ m_angle2 = currentAngle; }
    if( m_tickCounter == 3){ m_angle3 = currentAngle; m_tickCounter=0; }

    averageAngleList.remove(0);
    averageAngleList.add(currentAngle);

    // System.out.println("Angle: "+angle + "/t Speed: " + pSpeed);
    // SmartDashboard.putNumber("TargetSpeed", pSpeed);
    SmartDashboard.putNumber("angle", currentAngle);
    


  }
  
  public void constantSpeedBalance(double maxSpeed) {
    double pSpeed;
    double currentAngle = m_gyroSubsystem.getYAngle();

    pSpeed = Math.pow(((Math.abs(currentAngle))/15), 3);
    pSpeed = Math.copySign(pSpeed, currentAngle);
    pSpeed = pSpeed * -AutoBalanceConstants.HIGH_SPEED;
    pSpeed = MathUtil.clamp(pSpeed, -AutoBalanceConstants.HIGH_SPEED, AutoBalanceConstants.HIGH_SPEED);
    m_driveTrainSubsystem.constantSpeedDrive(pSpeed);
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pidController.reset();
    m_driveTrainSubsystem.curvatureDrive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // boolean finished = (m_pidController.atSetpoint());
    // boolean finished = (m_pidController.atSetpoint());
    // return finished;
    return false;
  }
}
