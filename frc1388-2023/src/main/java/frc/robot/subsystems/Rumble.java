// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Rumble extends SubsystemBase {
  private final CommandXboxController m_controller;
  private final Timer m_timer = new Timer();

  /** Creates a new Rumble. */
  public Rumble(CommandXboxController commandXboxController) {
    m_controller = commandXboxController;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
