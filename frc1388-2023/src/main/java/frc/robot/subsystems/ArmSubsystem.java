// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  public enum ArmSetPoint { // TODO add more positions
    retracted, extended
  }
  public final AnArmPosition m_extendedPos = new AnArmPosition(
    ArmConstants.WRIST_POSITION_UP,
    ArmConstants.PRIMARY_ARM_POSITION_UP
  );

  public final AnArmPosition m_retractedPos = new AnArmPosition(
    ArmConstants.WRIST_POSITION_DOWN,
    ArmConstants.PRIMARY_ARM_POSITION_DOWN
  );

  // currently not used
  private ArmSetPoint m_armSetPoint = ArmSetPoint.retracted;

  // mid arm
  private final CANSparkMax m_wristMotor;
  private final RelativeEncoder m_wristEncoder;
  private final DigitalInput m_wristLimitSwitch;
  
  // primary
  private final WPI_TalonFX m_primaryMotor;
  private final DigitalInput m_primaryArmLimitSwitch;

  /** Creates a new Arm. */
  public ArmSubsystem(CANSparkMax midArm, WPI_TalonFX primary, DigitalInput midArmLimit, DigitalInput primaryLimit) {
    m_wristMotor = midArm;
      m_wristMotor.setIdleMode(IdleMode.kBrake);
    m_wristEncoder = m_wristMotor.getEncoder();
    m_wristLimitSwitch = midArmLimit;

    m_primaryMotor = primary;
      m_primaryMotor.setNeutralMode(NeutralMode.Brake);
    m_primaryArmLimitSwitch = primaryLimit;
  }

  /** sets the power of the wrist motor */
  public void setWristMotorPower(double power) {
    if (
      (power < 0) && (!m_wristLimitSwitch.get())
      || (power > 0) && (m_wristEncoder.getPosition() < ArmConstants.WRIST_POSITION_MAX)
      || power == 0
    ) m_wristMotor.set(power);
  }

  /** sets the power of the primary motor */
  public void setPrimaryMotorPower(double power) {
    if (
      (power < 0) && (!m_primaryArmLimitSwitch.get())
      || (power > 0) && (!(m_primaryMotor.getSelectedSensorPosition() < ArmConstants.PRIMARY_ARM_POSITION_MAX))
      || power == 0
    ) m_primaryMotor.set(power);
  }

  /** position of mid arm in rotations */
  public void setWristMotorPosition(double position) {
    double distToSetPos = position - getPrimaryArmPosition();
    if (Math.abs(distToSetPos) > ArmConstants.DEADBAND){
      m_wristMotor.set(Math.copySign(0.5, distToSetPos));
    }
  }

  /** position of primary arm in rotations */
  public void setPrimaryMotorPosition(double position) {
    double distToSetPos = position - getWristPosition();
    if (Math.abs(distToSetPos) > ArmConstants.DEADBAND){
      m_primaryMotor.set(Math.copySign(0.5, distToSetPos));
    }
  }

  public void setArmPosition(ArmSetPoint setPoint) {
    m_armSetPoint = setPoint;
    switch (setPoint) {
      case retracted: {
        setWristMotorPosition(m_retractedPos.midArmPosition);
        setPrimaryMotorPower(m_retractedPos.primaryPosition);
        break;
      } 
      case extended: {
        setWristMotorPosition(m_extendedPos.midArmPosition);
        setPrimaryMotorPower(m_extendedPos.primaryPosition);
        break;
      }
    }
  }

  public void parallelArmSet(double speed) {
    setPrimaryMotorPower(speed);
    setWristMotorPosition(getPrimaryArmPosition() + ArmConstants.FLAT_TO_UP);
  }

  public void stowedArmSet(double speed) {
    setPrimaryMotorPower(speed);
    setWristMotorPosition(getPrimaryArmPosition() + ArmConstants.FLAT_TO_UP + 0.25);
  }

  public double getPrimaryArmPosition() {
    return m_primaryMotor.getSelectedSensorPosition() / ArmConstants.ENCODER_UNITS_PER_PRIMARY_ARM_ROTATIONS;
  }

  public double getWristPosition() {
    return m_wristEncoder.getPosition() / ArmConstants.WRIST_MOTOR_ROTATIONS_PER_WRIST_ARM_ROTATIONS;
  }

  // gets the wrist position relative to the ground
  public double getWristRelativePosition() {
    return -getWristPosition() + getPrimaryArmPosition() + ArmConstants.FLAT_TO_UP;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if (m_wristLimitSwitch.get()) m_wristEncoder.setPosition(ArmConstants.WRIST_POSITION_AT_LIMIT_SWITCH);
    if (m_primaryArmLimitSwitch.get()) m_primaryMotor.setSelectedSensorPosition(ArmConstants.PRIMARY_ARM_POSITION_AT_LIMIT_SWITCH);
  }

  public class AnArmPosition {
    // private final PossibleArmPositions m_thisArmPosition;
    private final double midArmPosition;
    private final double primaryPosition;

    AnArmPosition(double midArmPosition, double primaryArmPosition) {
      // m_thisArmPosition = thisPosition;
      this.midArmPosition = midArmPosition;
      this.primaryPosition = primaryArmPosition;
    }
  }
}
