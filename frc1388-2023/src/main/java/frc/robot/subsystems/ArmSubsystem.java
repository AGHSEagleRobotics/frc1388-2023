// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
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
  private final PIDController m_pidController = new PIDController(0.1, 0, 0);
  
  // primary
  private final WPI_TalonFX m_primaryMotor;
  private final DigitalInput m_primaryArmLimitSwitch;

  /** Creates a new Arm. */
  public ArmSubsystem(CANSparkMax midArm, WPI_TalonFX primary, DigitalInput midArmLimit, DigitalInput primaryLimit) {
    m_wristMotor = midArm;
      m_wristMotor.setIdleMode(IdleMode.kBrake);
      m_wristMotor.setInverted(false);
    m_wristEncoder = m_wristMotor.getEncoder();
    m_wristLimitSwitch = midArmLimit;

    m_primaryMotor = primary;
      m_primaryMotor.setInverted(true);
      m_primaryMotor.configFactoryDefault();
      // m_primaryMotor.getSelectedSensorPosition(0);
      m_primaryMotor.setNeutralMode(NeutralMode.Brake);
      m_primaryMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    m_primaryArmLimitSwitch = primaryLimit;
      // m_primaryArmLimitSwitch.
  }

  /** Sets the power of the wrist motor. The range of motion is limited by the limit switch and encoder 
   * @param power the power to set the motor [-1, 1]
  */
  public void setWristMotorPower(double power) {
    // if (
    //   (power < 0) && (!m_wristLimitSwitch.get())
    //   || (power > 0) && (getWristPosition() < ArmConstants.WRIST_POSITION_MAX)
    // ) {
    //   m_wristMotor.set(power);
    // } else {
    //   m_wristMotor.set(0);
    // }
    m_wristMotor.set(power);
  }

  /** Sets the power of the primary motor. The range of motion is limited by the limit switch and encoder
   * @param power the power to set the motor [-1, 1]
   */
  public void setPrimaryMotorPower(double power) {
    // SmartDashboard.putNumber("right y input > ", power);
    if (
      (power < 0) && (!isPrimaryLimitContacted())
      || (power > 0) && (getPrimaryArmPosition() < ArmConstants.PRIMARY_ARM_POSITION_MAX)
    ) {
      m_primaryMotor.set(power);
    } else {
      m_primaryMotor.set(0);
    }
  }

  @Deprecated
  /** doesn't work */
  public void setWristMotorSpeed(double speed) {
    m_wristMotor.set(m_pidController.calculate(m_wristEncoder.getVelocity(), speed));
  }

  /** position of mid arm in rotations */
  public void setWristMotorPosition(double position) {
    double distToSetPos = position - getPrimaryArmPosition();
    if (Math.abs(distToSetPos) > ArmConstants.DEADBAND){
      setWristMotorPower(Math.copySign(0.5, distToSetPos));
    }
  }

  /** position of primary arm in rotations */
  public void setPrimaryMotorPosition(double position) {
    double distToSetPos = position - getWristPosition();
    if (Math.abs(distToSetPos) > ArmConstants.DEADBAND){
      setPrimaryMotorPower(Math.copySign(0.5, distToSetPos));
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

  @Deprecated
  public void parallelArmSet(double speed) {
    setPrimaryMotorPower(speed);
    // setWristMotorPosition(getPrimaryArmPosition());
    // setWristMotorPosition(getPrimaryArmPosition() + ArmConstants.FLAT_TO_UP); // <-- doesn't work
  }

  @Deprecated //maybe ?
  public void stowedArmSet(double speed) {
    setPrimaryMotorPower(speed);
    setWristMotorPower(-1.0);
    // setWristMotorPosition(ArmConstants.FLAT_TO_UP + 0.25);
    // setWristMotorPosition(getPrimaryArmPosition() + ArmConstants.FLAT_TO_UP + 0.25); // <-- doesn't work
  }

  /**
   * the primary arm position is zeroed every time it hits the limit switch
   * @return the arm position in rotations, 0 is at the limit switch, positive is up
   */
  public double getPrimaryArmPosition() {
    return m_primaryMotor.getSelectedSensorPosition() / ArmConstants.ENCODER_UNITS_PER_PRIMARY_ARM_ROTATIONS;
  }

    /**
   * the wrist arm position is zeroed every time it hits the limit switch
   * @return the arm position in rotations, 0 is at the limit switch, positive is down and away from the limit switch
   */
  public double getWristPosition() {
    return m_wristEncoder.getPosition() / ArmConstants.WRIST_MOTOR_ROTATIONS_PER_WRIST_ARM_ROTATIONS;
  }
  
  /**
   * the limit switch on the primary arm
   * @return true if the arm is down, in contact with the limit switch, false if the arm is not in contact with the limit switch
   */
  private boolean isPrimaryLimitContacted() {
    return !m_primaryArmLimitSwitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if (m_wristLimitSwitch.get()) m_wristEncoder.setPosition(ArmConstants.WRIST_POSITION_AT_LIMIT_SWITCH);
    if (isPrimaryLimitContacted()) m_primaryMotor.setSelectedSensorPosition(ArmConstants.PRIMARY_ARM_POSITION_AT_LIMIT_SWITCH);
    SmartDashboard.putBoolean("primary limit switch", isPrimaryLimitContacted());
    // SmartDashboard.putBoolean("wrist limit switch", m_wristLimitSwitch.get());
    SmartDashboard.putNumber("primary arm", getPrimaryArmPosition());
    // SmartDashboard.putNumber("primary arm raw units", m_primaryMotor.getSelectedSensorPosition());
    // SmartDashboard.putNumber("wrist motor", m_wristEncoder.getPosition() / ArmConstants.WRIST_MOTOR_ROTATIONS_PER_WRIST_ARM_ROTATIONS);
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
