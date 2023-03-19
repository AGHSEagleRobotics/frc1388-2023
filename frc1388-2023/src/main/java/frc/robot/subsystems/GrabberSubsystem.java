// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.print.attribute.standard.PrinterMessageFromOperator;

import com.fasterxml.jackson.databind.util.PrimitiveArrayBuilder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GrabberConstants;

public class GrabberSubsystem extends SubsystemBase {
  private DataLog m_log = DataLogManager.getLog();


  public enum GrabberPosition{
    open(GrabberConstants.GRABBER_POSITION_OPEN), closed(GrabberConstants.GRABBER_POSITION_CLOSED);

    private final int value;

    private GrabberPosition(int value) {
      this.value = value;
    }

    public int get() {
      return value;
    }
  }
  private GrabberPosition m_grabberSetPosition = GrabberPosition.open;

  private final CANSparkMax m_grabberMotor;
  private final RelativeEncoder m_grabberEncoder;
  private final DigitalInput m_grabberLimit;

  private boolean m_hasEncoderBeenReset = false;

  // private double m_encoderOffset = 0;

  /** Creates a new Grabber. */
  public GrabberSubsystem(CANSparkMax motor, DigitalInput limitSwitch) {
    m_grabberMotor = motor;
      m_grabberMotor.setSmartCurrentLimit(GrabberConstants.SMART_CURRENT_LIMIT); // in amps
      m_grabberMotor.setIdleMode(IdleMode.kBrake);
      m_grabberMotor.setInverted(true);
    m_grabberEncoder = m_grabberMotor.getEncoder();
    m_grabberLimit = limitSwitch;
  }

  // @Deprecated
  // public void setGrabberPresetPosition(GrabberPosition position) {
  //   m_grabberSetPosition = position;
  //   double distToSetPoint = m_grabberSetPosition.get() - m_grabberEncoder.getPosition();
  //   if (Math.abs(distToSetPoint) > GrabberConstants.GRABBER_ENCODER_DEADBAND) {
  //     m_grabberMotor.set(Math.copySign(0.2, distToSetPoint));
  //   }
  // }

  public void setGrabberPosition(double position) {
    double distToSetPoint = position - m_grabberEncoder.getPosition();
    if (Math.abs(distToSetPoint) > GrabberConstants.GRABBER_ENCODER_DEADBAND) {
      m_grabberMotor.set(Math.copySign(0.2, distToSetPoint));
    }
  }

  public void setGrabberMotor(double power) {
    if (m_hasEncoderBeenReset) {
      m_grabberMotor.set(power);
    } else {
      m_grabberMotor.set(MathUtil.clamp(power, -1, 0));
    }
  }

  public void setGrabberEncoder(double value) {
    m_hasEncoderBeenReset = true;
    m_grabberEncoder.setPosition(value);
  }

  public double getGrabberEncoder() {
    return m_grabberEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if (m_grabberLimit.get()) m_grabberEncoder.setPosition(Constants.GrabberConstants.GRABBER_POSITION_AT_LIMIT_SWITCH);
    // System.out.println("grabber limit switch: " + m_grabberLimit.get());
    SmartDashboard.putNumber("grabber motor position ", m_grabberEncoder.getPosition());
    SmartDashboard.putNumber("grabber motor current ", m_grabberMotor.getOutputCurrent());
    if (!m_hasEncoderBeenReset && m_grabberMotor.getOutputCurrent() > 20);
    SmartDashboard.putBoolean("|||has the arm been reset?|||", m_hasEncoderBeenReset);
    // m_log.appendDouble(0, m_grabberEncoder.getPosition(), 0);
  }
}
