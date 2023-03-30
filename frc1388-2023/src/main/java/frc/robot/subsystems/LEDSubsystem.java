// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
  private double LED_RAINBOW = -0.99;
  private double LED_RED_PATTERN = 0.61;
  private double LED_BLUE_PATTERN = 0.87;

  public enum Color {
    red, blue
  }
  private Color m_color;
  
  public enum RampStates {
    upRamp, balanced
  }
  // private RampStates m_rampState;
  // private boolean m_balancing = false;
  private boolean m_balanced = false;

  private PWMSparkMax m_leds;

  /** Creates a new LED. */
  public LEDSubsystem(PWMSparkMax leds ) {
    m_leds = leds;
    m_leds.setSafetyEnabled(false);


  } // end constructor

  // public void hitDangerZone(){
  //   m_dangerZone = true;
  // }

  public void ledBalanced(){
    m_balanced = true;
    // m_rampState = RampStates.balanced;
  }

  // public boolean goLedUpRamp(){
  //   return false;
  // }

  public void ledNormal(){
    m_balanced = false;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // m_isBlue = (DriverStation.getAlliance() == DriverStation.Alliance.Blue);
    // m_isRed = (DriverStation.getAlliance() == DriverStation.Alliance.Red);

    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      m_color = Color.red;
    } else {
      m_color = Color.blue;
    }

    if (m_balanced) {
      m_leds.set(LED_RAINBOW);
    } else if (m_color == Color.blue) {
      m_leds.set(LED_BLUE_PATTERN);
    } else {
      m_leds.set(LED_RED_PATTERN);
    }

    // m_leds.set(LED_RAINBOW); // fun mode

    // if(m_isBlue){
    //   if(m_ledUpRamp = true){
    //     m_leds.set(LEDConstants.REDHEARTBEAT);
    //   }
    //   else if(m_ledBalanced){
    //     m_leds.set(LEDConstants.RAINBOW);
    //   }
    //   else{
    //     m_leds.set(LEDConstants.BLUE_LARSON);
    //   }
    // }
    // else if(m_isRed){
    //   if(m_ledUpRamp){
    //     m_leds.set(LEDConstants.REDHEARTBEAT);
    //   }
    //   else if(m_ledBalanced){
    //     m_leds.set(LEDConstants.RAINBOW);
    //   }
    //   else{
    //     m_leds.set(LEDConstants.RED_LARSON);
    //   }
    // }
    // else{
    //   if(m_ledUpRamp){
    //     m_leds.set(LEDConstants.REDHEARTBEAT);
    //   }
    //   else if(m_ledBalanced){
    //     m_leds.set(LEDConstants.RAINBOW);
    //   }
    //   else{
    //     m_leds.set(LEDConstants.BLUE_SOLID);
    //   }
    // }

  }
}