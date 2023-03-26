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
  private boolean m_dangerZone;
  private boolean m_ledBalanced;
  private boolean m_ledUpRamp;
  private boolean m_isBlue;
  private boolean m_isRed;

  /** Creates a new LED. */
  public LEDSubsystem(PWMSparkMax leds ) {
    m_leds = leds;
      m_leds.setSafetyEnabled(false);
  } // end constructor

  public void hitDangerZone(){
    m_dangerZone = true;
  }

  public void ledBalanced(){
    m_ledBalanced = true;
  }

  public void goLedUpRamp(){
    m_ledUpRamp = true;
  }

  public void ledNormal(){
    m_ledBalanced = false;
    m_ledUpRamp = false;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_isBlue = (DriverStation.getAlliance() == DriverStation.Alliance.Blue);
    m_isRed = (DriverStation.getAlliance() == DriverStation.Alliance.Red);

    if(m_isBlue){
      if(m_ledUpRamp){
        m_leds.set(LEDConstants.REDHEARTBEAT);
      }
      else if(m_ledBalanced){
        m_leds.set(LEDConstants.RAINBOW);
      }
      else{
        m_leds.set(LEDConstants.BLUE_LARSON);
      }
    }
    else if(m_isRed){
      if(m_ledUpRamp){
        m_leds.set(LEDConstants.REDHEARTBEAT);
      }
      else if(m_ledBalanced){
        m_leds.set(LEDConstants.RAINBOW);
      }
      else{
        m_leds.set(LEDConstants.RED_LARSON);
      }
    }
    else{
      if(m_ledUpRamp){
        m_leds.set(LEDConstants.REDHEARTBEAT);
      }
      else if(m_ledBalanced){
        m_leds.set(LEDConstants.RAINBOW);
      }
      else{
        m_leds.set(LEDConstants.BLUE_SOLID);
      }
    }

  }
}