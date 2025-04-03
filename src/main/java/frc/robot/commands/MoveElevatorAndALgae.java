// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorState;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.StreamDeckSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveElevatorAndALgae extends Command {
  /** Creates a new MoveElevatorAndALgae. */
  AlgaeArmSubsystem m_aas;
  ElevatorSubsystem m_es;
  ElevatorState constant;
  public MoveElevatorAndALgae(AlgaeArmSubsystem aas, ElevatorSubsystem es, ElevatorState constant) {
    m_aas = aas;
    m_es = es;
    this.constant = constant;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_aas.setArmPosition(constant.ALGAE_ARM_ANGLE);
    m_es.setElevatorTarget(constant.ELEVATOR_POSITION);
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
    // TODO: We want to add a if statement that is going to terminate the command when the elevator and the arm reaches the target pos
    if (m_aas.isAtPosition() && m_es.isAtPosition()) {
      return true;
    }
    return false;
  }
}
