// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GrabberConstants;

public class GrabberSubsystem extends SubsystemBase {
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

  // private double m_encoderOffset = 0;

  /** Creates a new Grabber. */
  public GrabberSubsystem(CANSparkMax motor, DigitalInput limitSwitch) {
    m_grabberMotor = motor;
      m_grabberMotor.setSmartCurrentLimit(20); // in amps
    m_grabberEncoder = m_grabberMotor.getEncoder();
    m_grabberLimit = limitSwitch;
  }

  public void setGrabberPosition(GrabberPosition position) {
    m_grabberSetPosition = position;
    double distToSetPoint = m_grabberSetPosition.get() - m_grabberEncoder.getPosition();

    if (Math.abs(distToSetPoint) > GrabberConstants.GRABBER_ENCODER_DEADBAND) {
      m_grabberMotor.set(Math.copySign(0.5, distToSetPoint));
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_grabberLimit.get()) m_grabberEncoder.setPosition(Constants.GrabberConstants.GRABBER_POSITION_AT_ENCODER);
  }
}
