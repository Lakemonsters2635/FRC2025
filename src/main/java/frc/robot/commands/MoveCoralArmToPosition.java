// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.CoralArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.StreamDeckSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveCoralArmToPosition extends Command {
  /** Creates a new MoveCoralArmToPosition. */

  CoralArmSubsystem m_cas;
  StreamDeckSubsystem m_sds;
  AlgaeArmSubsystem m_aas;
  ElevatorSubsystem m_es;
  Constants.ElevatorState constant;
  public MoveCoralArmToPosition(CoralArmSubsystem cas, StreamDeckSubsystem sds, AlgaeArmSubsystem aas, ElevatorSubsystem es) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_cas = cas;
    m_sds = sds;
    m_aas = aas;
    m_es = es;
    constant = Constants.E_STATE_CORAL_SOURCE;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    try{
      String level = m_sds.getCorralInfo()[2];
      SmartDashboard.putString("level", level);
      // switch (level) {
      //   case Constants.SD_SOURCE:
      //     // constant = Constants.E_STATE_CLIMB;
      //     constant = Constants.E_STATE_ALGAE_PROCESSOR;
      //     break;
        
      //   case Constants.SD_REEF_LEVEL_1:
      //     constant = Constants.E_STATE_ALGAE_PICKUP_GROUND;
      //     break;
    
      //   case Constants.SD_REEF_LEVEL_2:
      //     constant = Constants.E_STATE_ALGAE_PICKUP;
      //     break;
      
      //   case Constants.SD_REEF_LEVEL_3:
      //     constant = Constants.E_STATE_ALGAE_LOW;
      //     break;
      
      //   case Constants.SD_REEF_LEVEL_4:
      //     constant = Constants.E_STATE_ALGAE_HIGH;
      //     break;
          
      
      //   default:
      //     System.out.println("DEFAULT VALUE !!!!!!!!!!!!!!!");
      //     break;
      // }
      switch (level) {
        case Constants.SD_SOURCE:
          constant = Constants.E_STATE_CORAL_SOURCE;
          break;
        
        case Constants.SD_REEF_LEVEL_1:
          constant = Constants.E_STATE_CORAL_REEF_1;
          break;
    
        case Constants.SD_REEF_LEVEL_2:
          constant = Constants.E_STATE_CORAL_REEF_2;
          break;
      
        case Constants.SD_REEF_LEVEL_3:
          constant = Constants.E_STATE_CORAL_REEF_3;
          break;
      
        case Constants.SD_REEF_LEVEL_4:
          constant = Constants.E_STATE_CORAL_REEF_4;
          break;
      
        default:
          System.out.println("DEFAULT VALUE !!!!!!!!!!!!!!!");
          break;
      }
    } catch(Exception e){
      System.out.println(e);
      System.out.println("Exception !!!!!!!!!!!!!!!!!!!!!!!!!!");
    }

    m_cas.setPoseTarget(constant.CORAL_ARM_ANGLE);
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
