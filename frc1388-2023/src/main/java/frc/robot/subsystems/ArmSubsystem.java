// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  public enum ArmSetPoint { // TODO add more positions
    retracted, extended
  }
  public final AnArmPosition m_extendedPos = new AnArmPosition(
    ArmConstants.MID_ARM_POSITION_UP,
    ArmConstants.PRIMARY_ARM_POSITION_UP
  );

  public final AnArmPosition m_retractedPos = new AnArmPosition(
    ArmConstants.MID_ARM_POSITION_DOWN,
    ArmConstants.PRIMARY_ARM_POSITION_DOWN
  );

  // currently not used
  private ArmSetPoint m_armSetPoint = ArmSetPoint.retracted;

  // mid arm
  private final CANSparkMax m_midArmMotor;
  private final RelativeEncoder m_midArmEncoder;
  private final DigitalInput m_primaryArmLimitSwitch;

  // primary
  private final WPI_TalonFX m_primaryMotor;
  private final DigitalInput m_midArmLimitSwitch;

  /** Creates a new Arm. */
  public ArmSubsystem(CANSparkMax midArm, WPI_TalonFX primary, DigitalInput midArmLimit, DigitalInput primaryLimit) {
    m_midArmMotor = midArm;
    m_midArmEncoder = m_midArmMotor.getEncoder();
    m_midArmLimitSwitch = midArmLimit;

    m_primaryMotor = primary;
    m_primaryArmLimitSwitch = primaryLimit;
  }

  public void setMidArmMotor(double position) {
    double distToSetPos = position - m_midArmEncoder.getPosition();
    if (Math.abs(distToSetPos) > ArmConstants.DEADBAND) m_midArmMotor.set(Math.copySign(0.5, distToSetPos));
  }

  public void setPrimaryMotor(double position) {
    double distToSetPos = position - m_primaryMotor.getSelectedSensorPosition();
    if (Math.abs(distToSetPos) > ArmConstants.DEADBAND) m_primaryMotor.set(Math.copySign(0.5, distToSetPos));
  }

  public void setArmPosition(ArmSetPoint setPoint) {
    m_armSetPoint = setPoint;
    switch (setPoint) {
      case retracted: {
        setMidArmMotor(m_retractedPos.midArmPosition);
        setPrimaryMotor(m_retractedPos.primaryPosition);
        break;
      } 
      case extended: {
        setMidArmMotor(m_extendedPos.midArmPosition);
        setPrimaryMotor(m_extendedPos.primaryPosition);
        break;
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_midArmLimitSwitch.get()) m_midArmEncoder.setPosition(ArmConstants.MID_ARM_POSITION_AT_ENCODER);
    if (m_primaryArmLimitSwitch.get()) m_primaryMotor.setSelectedSensorPosition(ArmConstants.PRIMARY_ARM_POSITION_AT_ENCODER);
  }

  public class AnArmPosition {
    // private final PossibleArmPositions m_thisArmPosition;
    private final int midArmPosition;
    private final int primaryPosition;

    AnArmPosition(int midArmPosition, int primaryArmPosition) {
      // m_thisArmPosition = thisPosition;
      this.midArmPosition = midArmPosition;
      this.primaryPosition = primaryArmPosition;
    }
  }
}
