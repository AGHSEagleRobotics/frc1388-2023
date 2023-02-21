// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoBalanceConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.GyroSubsystem;

public class AutoBalance extends CommandBase {

  private enum BalanceStates {
    approachingRamp,
    goingUpRamp,
    rampTiltingDown,
    overshooting
  }

  private BalanceStates m_balanceState = BalanceStates.approachingRamp;

  private DriveTrainSubsystem m_driveTrainSubsystem;
  private GyroSubsystem m_gyroSubsystem;

  private double m_angle1 = 0;
  private double m_angle2 = 0;
  private double m_angle3 = 0;
  private int m_tickCounter = 0;

  private final PIDController m_pidController = new PIDController(0.02, 0, 0);

  /** Creates a new AutoTurn. */
  public AutoBalance(DriveTrainSubsystem driveTrainSubsystem, GyroSubsystem gyroSubsystem) {
    m_driveTrainSubsystem = driveTrainSubsystem;
    m_gyroSubsystem = gyroSubsystem;
    //System.out.println("*****************TURNCONSTUCTOR****************************************TURNCONSTRUCTOR*******************");
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrainSubsystem);
    //m_pidController.se  tTolerance(0.02);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // m_driveTrainSubsystem.resetGyro();
   // m_driveTrainSubsystem.setDeadbandZero();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //  double turnSpeed;
    m_tickCounter++;

    double pSpeed;
    double angle = m_gyroSubsystem.getYAngle();
    double averageAngle = Math.abs((m_angle1 + m_angle2 + m_angle3) / 3);
    // System.out.println("Angle1: "+ m_angle1 + "   Angle2: " + m_angle2 + "   Angle3: " + m_angle3);
    // System.out.println("Average Angle: "+ averageAngle);
  
    if( averageAngle - Math.abs(angle) > 1){
    //if( Math.abs(angle) < 13.5 ){
      pSpeed = 0;
    }
    else {
          //double pSpeed = angle * DriveTrainConstants.0.02;
      pSpeed = Math.pow(((Math.abs(angle))/15), 3);
      // System.out.println("(abs(angle)/15)^2.5 :  " + pSpeed);
      pSpeed = Math.copySign(pSpeed, angle);
      // System.out.println("pSpeed copySign :  " + pSpeed);
      pSpeed = pSpeed * -AutoBalanceConstants.HIGH_SPEED;
      pSpeed = MathUtil.clamp(pSpeed, -AutoBalanceConstants.HIGH_SPEED, AutoBalanceConstants.HIGH_SPEED);
      // System.out.println("pSpeed :  " + pSpeed);
      m_driveTrainSubsystem.constantSpeedDrive(pSpeed);
      // m_driveTrainSubsystem.constantSpeedDrive(12);
    
    }
    
    /* TODO test constant code
    double constantSpeed = DriveTrainConstants.BALANCE_CONSTANT;
    m_driveTrainSubsystem.tankDrive(constantSpeed, constantSpeed);
    */
    if( m_tickCounter == 1){ m_angle1 = angle; }
    if( m_tickCounter == 2){ m_angle2 = angle; }
    if( m_tickCounter == 3){ m_angle3 = angle; m_tickCounter=0; }

    // System.out.println("Angle: "+angle + "/t Speed: " + pSpeed);
    // SmartDashboard.putNumber("TargetSpeed", pSpeed);
    SmartDashboard.putNumber("angle", angle);
    


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
    boolean finished = (m_pidController.atSetpoint());
    // boolean finished = (m_pidController.atSetpoint());
    // return finished;
    return false;
  }
}
