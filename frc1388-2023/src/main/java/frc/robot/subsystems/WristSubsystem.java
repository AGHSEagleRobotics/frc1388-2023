// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class WristSubsystem extends SubsystemBase {

  // wrist
  private final WPI_TalonFX m_wristMotor;
  private final DigitalInput m_wristLimitSwitch;


  /** Creates a new Arm. */
  public WristSubsystem(WPI_TalonFX wristMotor, DigitalInput wristLimit) {
    m_wristMotor = wristMotor;
    m_wristMotor.setNeutralMode(NeutralMode.Brake);
    m_wristMotor.setInverted(false);
    m_wristMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, ArmConstants.WRIST_CURRENT_LIMIT, ArmConstants.WRIST_CURRENT_LIMIT,0));

    m_wristLimitSwitch = wristLimit;
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

  @Deprecated
  /** doesn't work */
  public void setWristMotorSpeed(double speed) {
    // m_wristMotor.set(m_pidController.calculate(m_wristMotor.getSelectedSensorVelocity(), speed));
  }

  /** position of wrist in rotations */
  public boolean setWristMotorPosition(double position) {
    double distToSetPos = position - getWristPosition();
    if (Math.abs(distToSetPos) > ArmConstants.DEADBAND){
      setWristMotorPower(Math.copySign(0.5, distToSetPos)); //TODO add constants for 0.5
      return false;
    } else {
      return true;
    }
  }

  @Deprecated //maybe ?
  public void stowedArmSet(double speed) {
    // setPrimaryArmMotorPower(speed);
    setWristMotorPower(-1.0);
    // setWristMotorPosition(ArmConstants.FLAT_TO_UP + 0.25);
    // setWristMotorPosition(getPrimaryArmPosition() + ArmConstants.FLAT_TO_UP + 0.25); // <-- doesn't work
  }

    /**
   * the wrist arm position is zeroed every time it hits the limit switch
   * @return the arm position in rotations, 0 is at the limit switch, positive is down and away from the limit switch
   */
  public double getWristPosition() {
    return m_wristMotor.getSelectedSensorPosition() / ArmConstants.WRIST_MOTOR_ROTATIONS_PER_WRIST_ARM_ROTATIONS;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run


    // if (m_wristLimitSwitch.get()){
      // m_wristEncoder.setPosition(ArmConstants.WRIST_POSITION_AT_LIMIT_SWITCH);
    // }

    SmartDashboard.putNumber("wrist motor position ", getWristPosition());
  }
}
