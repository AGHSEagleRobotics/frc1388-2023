// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {

  private PWMSparkMax m_leds;
  private boolean m_maxArm;
  private boolean m_ledBalanced;

  /** Creates a new LED. */
  public LEDSubsystem(PWMSparkMax leds ) {
    m_leds = leds;
      m_leds.setSafetyEnabled(false);
  } // end constructor

  public void hitMaxArm(){
    m_maxArm = true;
  }

  public void ledBalanced(){
    m_ledBalanced = true;
  }

  public void ledNormal(){
    m_ledBalanced = false;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(m_ledBalanced){
      m_leds.set(LEDConstants.RAINBOW);
    }
  }
}