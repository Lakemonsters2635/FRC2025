// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeIntakeOutCommand extends Command {
  /** Creates a new AlgaeIntakeOutCommand. */
  private AlgaeIntakeSubsystem m_algaeIntakeSubsystem;
  // private Joystick rJoystick = new Joystick(1);
  public AlgaeIntakeOutCommand(AlgaeIntakeSubsystem algaeIntakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_algaeIntakeSubsystem = algaeIntakeSubsystem;
    addRequirements(m_algaeIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_algaeIntakeSubsystem.outAlgaeIntake(Constants.ALGAE_INTAKE_VOLTAGE_OUT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_algaeIntakeSubsystem.stopAlgaeIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
