// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Dashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.GrabberConstants;
import frc.robot.RobotContainer.ArmGrabberClass;

public class GrabberSubsystem extends SubsystemBase {  

  private final CANSparkMax m_grabberMotor;
  private final RelativeEncoder m_grabberEncoder;

  private final Dashboard m_Dashboard;

  private final ArmGrabberClass m_armGrabberClass;

  private double m_grabberPower = 0;

  /** Creates a new Grabber. */
  public GrabberSubsystem(CANSparkMax motor, Dashboard dashboard, ArmGrabberClass armGrabberClass) {
    m_grabberMotor = motor;
    m_grabberMotor.setSmartCurrentLimit(GrabberConstants.SMART_CURRENT_LIMIT); // in amps
    m_grabberMotor.setIdleMode(IdleMode.kBrake);
    m_grabberMotor.setInverted(false);

    m_grabberEncoder = m_grabberMotor.getEncoder();

    m_Dashboard = dashboard;
    
    m_armGrabberClass = armGrabberClass;
  }

  public boolean setGrabberPosition(double position) {
    double distToSetPoint = position - m_grabberEncoder.getPosition();
    if (Math.abs(distToSetPoint) > GrabberConstants.GRABBER_ENCODER_DEADBAND) {
      m_grabberMotor.set(Math.copySign(0.2, distToSetPoint)); // TODO fix magic number
      return false;
    } else {
      return true;
    }
  }

  /**
   * sets the power of the grabber motor
   * @param power range [-1, 1] positive is out, negative is in
   */
  public void setGrabberMotor(double power) {
    m_grabberPower = power;
  }

  public void setGrabberEncoder(double value) {
    m_grabberEncoder.setPosition(value);
  }

  public double getGrabberEncoder() {
    return m_grabberEncoder.getPosition();
  }

  public double maxGrabberPosition() { //based on arm 
    double grabPosition = m_armGrabberClass.primaryArmPosition * (216.374 - (1131.87 * m_armGrabberClass.primaryArmPosition) );
    grabPosition = MathUtil.clamp(grabPosition, GrabberConstants.GRABBER_MAX_AT_FULL_ARM, GrabberConstants.GRABBER_MAX_EXTENSION); //9 is max for grabber
    return grabPosition;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    
    m_armGrabberClass.grabberPosition = getGrabberEncoder();

    if (m_armGrabberClass.hasGrabberEncoderBeenReset) {
      if ((m_armGrabberClass.primaryArmPosition > ArmConstants.ARM_MAX_EXTEND_LOW) && (m_armGrabberClass.grabberPosition >= maxGrabberPosition())) {
        m_grabberMotor.set(GrabberConstants.GRABBER_POWER_IN);
          } else {
        m_grabberMotor.set(m_grabberPower);
      }
    } else {
      if (m_armGrabberClass.primaryArmPosition > ArmConstants.ARM_MAX_EXTEND_LOW) {
        m_grabberMotor.setSmartCurrentLimit(10);
        m_grabberMotor.set(GrabberConstants.GRABBER_LOW_POWER_IN);
      } else {
        m_grabberMotor.setSmartCurrentLimit(GrabberConstants.SMART_CURRENT_LIMIT);
        m_grabberMotor.set(m_grabberPower);
        // m_grabberMotor.set(MathUtil.clamp(power, -1, 0));
        
      }
    }
    
    SmartDashboard.putNumber("max grabber position", maxGrabberPosition());
    SmartDashboard.putNumber("grabber position", m_armGrabberClass.grabberPosition);
    SmartDashboard.putNumber("grabber motor current", m_grabberMotor.getOutputCurrent());
    SmartDashboard.putBoolean("has grabber been reset", m_armGrabberClass.hasGrabberEncoderBeenReset);

    m_Dashboard.setIfGrabberReset(m_armGrabberClass.hasGrabberEncoderBeenReset);
  }
}
