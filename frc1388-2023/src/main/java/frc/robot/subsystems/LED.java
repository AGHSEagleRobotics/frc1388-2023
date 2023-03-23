// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

  private PWMSparkMax m_leds;
  private boolean maxArm;
  
  /** Creates a new LED. */
  public LED(PWMSparkMax leds ) {
    m_leds = leds;
      m_leds.setSafetyEnabled(false);
  } // end constructor




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}