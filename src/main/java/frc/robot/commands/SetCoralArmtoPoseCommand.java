// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralArmSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetCoralArmtoPoseCommand extends Command {
  private CoralArmSubsystem m_coralArmSubsystem;
  double m_poseTarget;
  /** Creates a new SetCoralArmtoPoseCommand. */
  public SetCoralArmtoPoseCommand(CoralArmSubsystem coralArmSubsystem, double poseTarget) {
    m_coralArmSubsystem = coralArmSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_coralArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_coralArmSubsystem.setPoseTarget(m_poseTarget);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((m_coralArmSubsystem.getDegrees() > (m_poseTarget - 2)) && (m_coralArmSubsystem.getDegrees() < (m_poseTarget + 2))) {
      return true;
    }
    return false;
  }
}
