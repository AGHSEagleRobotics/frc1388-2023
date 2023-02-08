// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScriptable extends SequentialCommandGroup {

  /** Creates a new AutoScriptable. */
  public AutoScriptable() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands( new AutoMove(0, 0), new AutoTurn(0, 0));

    // m_objectiveChooser.addOption( "Testing Auto Option", testAuto );
    // m_objectiveChooser.setDefaultOption( "Testing Auto Option", testAuto );
    // String objective = m_objectiveChooser.getSelected();

    // switch(objective) {
    //     case "testAuto":
    //     sequence(new AutoMove(0, 0));
    //     break;
    //     default: 
    //     break;
    }
  }
