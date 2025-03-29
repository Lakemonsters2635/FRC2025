// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.StreamDeckSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveAlgaeToPose extends Command {
  /** Creates a new MoveAlgaeToPose. */
  StreamDeckSubsystem m_sds;
  AlgaeArmSubsystem m_aas;
  ElevatorSubsystem m_es;

  Constants.ElevatorState constant;
  public MoveAlgaeToPose(StreamDeckSubsystem sds, AlgaeArmSubsystem aas, ElevatorSubsystem es) {
    m_sds = sds;
    m_aas = aas;
    m_es = es;
    constant = Constants.E_STATE_CORAL_SOURCE;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    try{
      String level = m_sds.getAlgaeInfo();
      SmartDashboard.putString("algaeLevel", level);
      switch (level) {
        case Constants.SD_ALGAE_HIGH:
          constant = Constants.E_STATE_ALGAE_HIGH;
          break;
        
        case Constants.SD_ALGAE_LOW:
          constant = Constants.E_STATE_ALGAE_LOW;
          break;
    
        case Constants.SD_ALGAE_CORRAL:
          constant = Constants.E_STATE_ALGAE_PICKUP;
          break;
      
        case Constants.SD_ALGAE_GROUND:
          constant = Constants.E_STATE_ALGAE_PICKUP_GROUND;
          break;
      
        case Constants.SD_ALGAE_PROCESS:
          constant = Constants.E_STATE_ALGAE_PROCESSOR;
          break;
          
        case Constants.SD_ALGAE_BARGE:
          constant = Constants.E_STATE_ALGAE_BARGE;
          break;
      
        default:
          System.out.println("DEFAULT VALUE !!!!!!!!!!!!!!!");
          break;
      }
    } catch(Exception e){
      System.out.println(e);
      System.out.println("Exception !!!!!!!!!!!!!!!!!!!!!!!!!!");
    }

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
    return true;
  }
}
