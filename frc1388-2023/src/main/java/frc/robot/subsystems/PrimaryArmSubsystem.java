// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.GrabberConstants;
import frc.robot.RobotContainer.ArmGrabberClass;

public class PrimaryArmSubsystem extends SubsystemBase {

  // primary
  private final WPI_TalonFX m_primaryMotor;
  private final DigitalInput m_primaryArmLimitSwitch;

  private final ArmGrabberClass m_armGrabberClass;

  private double m_primaryArmPower = 0;

  private boolean m_lastLimitValue = false;

  /** Creates a new Arm. */
  public PrimaryArmSubsystem(WPI_TalonFX primary, DigitalInput primaryLimit, ArmGrabberClass armGrabberClass) {

    m_primaryMotor = primary;
    m_primaryMotor.configFactoryDefault();
    m_primaryMotor.setInverted(true);
    m_primaryMotor.setNeutralMode(NeutralMode.Brake);
    m_primaryMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_primaryMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, ArmConstants.PRIMARY_ARM_CURRENT_LIMIT, ArmConstants.PRIMARY_ARM_CURRENT_LIMIT, 0));

    m_primaryArmLimitSwitch = primaryLimit;

    m_armGrabberClass = armGrabberClass;

    // m_lastLimitValue = isPrimaryLimitContacted();
  }

  /**
   * sets the power of the primary arm motor
   * @param power range [-1, 1], positive makes the arm go up, negative retracts the arm
   */
  public void setPrimaryArmMotorPower(double power) {
    m_primaryArmPower = power;
  }

  /** position of primary arm in rotations */
  public boolean setPrimaryArmMotorPosition(double position) {
    double distToSetPos = position - getPrimaryArmPosition();
    if (Math.abs(distToSetPos) > ArmConstants.DEADBAND) {
      setPrimaryArmMotorPower(Math.copySign(0.5, distToSetPos)); //TODO add constants for 0.5
      return false;
    } else {
      return true;
    }
  }

  /**
   * the primary arm position is zeroed every time it hits the limit switch
   * @return the arm position in rotations, 0 is at the limit switch, positive is up
   */
  public double getPrimaryArmPosition() {
    return m_primaryMotor.getSelectedSensorPosition() / ArmConstants.ENCODER_UNITS_PER_PRIMARY_ARM_ROTATIONS;
  }

  public double maxArmPosition() {
    double armPosition = ( 10*( Math.sqrt(3)*Math.sqrt(421875 - (37729 * m_armGrabberClass.grabberPosition)) + 1125) )/113187;
    if ( m_armGrabberClass.grabberPosition <= GrabberConstants.GRABBER_MAX_AT_FULL_ARM )
    {
      armPosition = ArmConstants.ARM_MAX_EXTEND_HIGH;
    }
    return armPosition;
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


    m_armGrabberClass.primaryArmPosition = getPrimaryArmPosition();

    if (
      ((m_primaryArmPower < 0) && (!isPrimaryLimitContacted()))
      || ((m_primaryArmPower > 0) && (getPrimaryArmPosition() < ArmConstants.ARM_MAX_EXTEND_HIGH))
      || (m_primaryArmPower == 0)
    ) {
      if ((getPrimaryArmPosition() > ArmConstants.ARM_MAX_EXTEND_LOW) 
      && (getPrimaryArmPosition() > maxArmPosition() ) 
      && m_armGrabberClass.hasGrabberEncoderBeenReset
      && (m_primaryArmPower > 0)) {
        m_primaryMotor.set(0);
      } else {
        m_primaryMotor.set(m_primaryArmPower);
      }
    } else {
      m_primaryMotor.set(0);
    }

    /// XXX important debug, remove
    // System.out.println(m_primaryArmPower);
    // m_primaryMotor.set(m_primaryArmPower);
    boolean primaryLimitContacted = isPrimaryLimitContacted();
    if (primaryLimitContacted && !m_lastLimitValue) {
      m_primaryMotor.setSelectedSensorPosition(ArmConstants.PRIMARY_ARM_POSITION_AT_LIMIT_SWITCH);
    }
    m_lastLimitValue = primaryLimitContacted;

    SmartDashboard.putNumber("max arm position", maxArmPosition());
    SmartDashboard.putNumber("primary arm position", getPrimaryArmPosition());
    SmartDashboard.putBoolean("primary limit switch", isPrimaryLimitContacted());
    SmartDashboard.putNumber("primary arm power", m_primaryArmPower);
  }
}
