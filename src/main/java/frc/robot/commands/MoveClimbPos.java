// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.CoralArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.StreamDeckSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveClimbPos extends Command {
  /** Creates a new MoveClimbPos. */
  
  CoralArmSubsystem m_cas;
  StreamDeckSubsystem m_sds;
  AlgaeArmSubsystem m_aas;
  ElevatorSubsystem m_es;
  Constants.ElevatorState constant;

  public MoveClimbPos(CoralArmSubsystem cas, StreamDeckSubsystem sds, AlgaeArmSubsystem aas, ElevatorSubsystem es) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_cas = cas;
    m_sds = sds;
    m_aas = aas;
    m_es = es;
    constant = Constants.E_STATE_CLIMB;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_cas.setPoseTarget(constant.CORAL_ARM_ANGLE);
    m_aas.setArmPosition(constant.ALGAE_ARM_ANGLE);
    m_es.setElevatorTarget(constant.ELEVATOR_POSITION);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_es.isPIDControl = false;
    m_es.unspoolElevator.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_es.isAtPosition()){
      return true;
    }
    return false;
  }
}
