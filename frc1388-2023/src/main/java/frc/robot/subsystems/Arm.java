// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstatns;

public class Arm extends SubsystemBase {
  public enum ArmPositions { // TODO add more positions
    retracted, extended
  }
  private ArmPositions m_armPosition = ArmPositions.retracted;
  private final CANSparkMax m_midArmMotor;
  private final RelativeEncoder m_midArmEncoder;
  private final DigitalInput m_primaryArmLimitSwitch;

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

  public void setPrimaryMotor(double position) {
    double distToSetPos = position - m_primaryMotor.getSelectedSensorPosition();
    if (Math.abs(distToSetPos) > ArmConstatns.DEADBAND) m_primaryMotor.set(Math.copySign(0.5, distToSetPos));
  }
  
  public void setMidArmMotor(double position) {
    double distToSetPos = position - m_midArmEncoder.getPosition();
    if (Math.abs(distToSetPos) > ArmConstatns.DEADBAND) m_midArmMotor.set(Math.copySign(0.5, distToSetPos));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_midArmLimitSwitch.get()) m_midArmEncoder.setPosition(0);
    if (m_primaryArmLimitSwitch.get()) m_primaryMotor.setSelectedSensorPosition(0);
  }
}
