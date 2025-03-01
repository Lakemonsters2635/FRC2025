// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.StreamDeckSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunAutoCommand extends SequentialCommandGroup {
  /** Creates a new RunAutoCommand. */
  Autos m_autos;
  public RunAutoCommand(StreamDeckSubsystem m_sds) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_autos = RobotContainer.m_autos;
    Command autoCommand = m_autos.sourceLineup();
    String val = m_sds.getCorralInfo()[0];
    switch (val) {
      case "L":
        autoCommand = m_autos.reefCorralLeft();
        break;
      case "R":
        autoCommand = m_autos.reefCorralRight();
        break;
      case "1":
       autoCommand = m_autos.sourceLineup();
       break;
      case "2":
        autoCommand = m_autos.reefAlgae();
      default:
        break;
    }
    autoCommand = m_autos.closestAprilTag();
    addCommands(autoCommand);
  }
}
