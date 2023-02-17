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
// import frc.robot.subsystems.Arm.ArmPositions.ArmSetPoint;
// import frc.robot.subsystems.Arm.ArmPositions.MotorSetPoint;

public class Arm extends SubsystemBase {
  public enum PossibleArmPositions { // TODO add more positions
    retracted, extended
  }
  AnArmPosition m_extendedPos = new AnArmPosition(
    PossibleArmPositions.extended,
    ArmConstants.MID_ARM_POSITION_UP,
    ArmConstants.PRIMARY_ARM_POSITION_UP
  );

  AnArmPosition m_retractedPos = new AnArmPosition(
    PossibleArmPositions.retracted,
    ArmConstants.MID_ARM_POSITION_DOWN,
    ArmConstants.PRIMARY_ARM_POSITION_DOWN
  );

  private PossibleArmPositions m_armPosition = PossibleArmPositions.retracted;

  // private final ArmPositions m_ArmPositions = new ArmPositions(
  //   ArmConstants.MID_ARM_POSITION_UP,
  //   ArmConstants.MID_ARM_POSITION_UP,
  //   ArmConstants.PRIMARY_ARM_POSITION_DOWN,
  //   ArmConstants.PRIMARY_ARM_POSITION_UP
  // );

  // mid arm
  private final CANSparkMax m_midArmMotor;
  private final RelativeEncoder m_midArmEncoder;
  private final DigitalInput m_primaryArmLimitSwitch;

  // primary
  private final WPI_TalonFX m_primaryMotor;
  private final DigitalInput m_midArmLimitSwitch;

  /** Creates a new Arm. */
  public Arm(CANSparkMax midArm, WPI_TalonFX primary, DigitalInput midArmLimit, DigitalInput primaryLimit) {
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

  public void setArmPosition(PossibleArmPositions setPoint) {
    m_armPosition = setPoint;
    // setMidArmMotor(m_ArmPositions.getMotorSetPoint(MotorSetPoint.midArm));
    // setPrimaryMotor(m_ArmPositions.getMotorSetPoint(MotorSetPoint.primary));

    switch(m_armPosition) {
      case extended: {
        setMidArmMotor(m_retractedPos.m_midArmPosition);
        setPrimaryMotor(m_retractedPos.m_primaryPosition);
      }
    }
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_midArmLimitSwitch.get()) m_midArmEncoder.setPosition(ArmConstants.MID_ARM_POSITION_AT_ENCODER);
    if (m_primaryArmLimitSwitch.get()) m_primaryMotor.setSelectedSensorPosition(ArmConstants.PRIMARY_ARM_POSITION_AT_ENCODER);
  }

  // public static class ArmPositions {
  //   public enum ArmSetPoint {
  //     low, high
  //   }
  //   ArmSetPoint m_ArmSetPoint = ArmSetPoint.low;

  //   public enum MotorSetPoint {
  //     midArm, primary
  //   }

  //   private final int m_midArmLow;
  //   private final int m_midArmHigh;
  //   private final int m_primaryLow;
  //   private final int m_primaryHigh;

  //   public ArmPositions(int midArmLow, int midArmHigh, int primaryLow, int primaryHigh) {
  //     m_midArmLow = midArmLow;
  //     m_midArmHigh = midArmHigh;
  //     m_primaryLow = primaryLow;
  //     m_primaryHigh = primaryHigh;
  //   }

  //   public void setSetPoint(ArmSetPoint setPoint){
  //     m_ArmSetPoint = setPoint;
  //   }

  //   public int getMotorSetPoint(MotorSetPoint motorSetPoint) {
  //     if (m_ArmSetPoint == ArmSetPoint.low && motorSetPoint == MotorSetPoint.midArm) return m_midArmLow;
  //     if (m_ArmSetPoint == ArmSetPoint.high && motorSetPoint == MotorSetPoint.midArm) return m_midArmHigh;
  //     if (m_ArmSetPoint == ArmSetPoint.low && motorSetPoint == MotorSetPoint.primary) return m_primaryLow;
  //     if (m_ArmSetPoint == ArmSetPoint.high && motorSetPoint == MotorSetPoint.primary) return m_primaryHigh;
  //     return 0;
  //   }
  // }

  public class AnArmPosition {
    private final PossibleArmPositions m_thisArmPosition;
    private final int m_midArmPosition;
    private final int m_primaryPosition;

    AnArmPosition(PossibleArmPositions thisPosition, int midarmpos, int primaryArmPositon){
      m_thisArmPosition = thisPosition;
      m_midArmPosition = midarmpos;
      m_primaryPosition = primaryArmPositon;
    }

    public PossibleArmPositions getThisArmPosition() {
      return m_thisArmPosition;
    }

    public int getMidArmPos() {
      return m_midArmPosition;
    }

    public int getPrimaryPos() {
      return m_primaryPosition;
    }
  }
}
