
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlgaeIntakeInCommand;
import frc.robot.commands.AlgaeIntakeOutCommand;
import frc.robot.commands.AlgaeProcessorCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.ClimberDownCommand;
import frc.robot.commands.ClimberUpCommand;
import frc.robot.commands.ElevatorDownCommand;
import frc.robot.commands.ElevatorUpCommand;
import frc.robot.commands.MoveAlgaeToPose;
import frc.robot.commands.MoveClimbPos;
import frc.robot.commands.RunAutoCommand;
import frc.robot.commands.VisionAutoCommand;
import frc.robot.commands.VisionPureAutoCommand;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ObjectTrackerSubsystem;
import frc.robot.subsystems.StreamDeckSubsystem;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Joysticks
  public static Joystick rightJoystick = new Joystick(Constants.RIGHT_JOYSTICK_CHANNEL);
  public static Joystick leftJoystick = new Joystick(Constants.LEFT_JOYSTICK_CHANNEL);

  // Subsystems
  public static final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  public static final AlgaeIntakeSubsystem m_algaeIntakeSubsystem = new AlgaeIntakeSubsystem();
  // public static final CoralIntakeSubsystem m_coralIntakeSubsystem = new CoralIntakeSubsystem();
  public static final AlgaeArmSubsystem m_algaeArmSubsystem = new AlgaeArmSubsystem();
  public static final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  // public static final CoralArmSubsystem m_coralArmSubsystem = new CoralArmSubsystem();
  public static final StreamDeckSubsystem m_streamDeckSubsystem = new StreamDeckSubsystem();
  public static final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  public static final ObjectTrackerSubsystem m_objectTrackerSubsystem = new ObjectTrackerSubsystem("front");

  // Commands
  public static final AlgaeIntakeInCommand m_algaeIntakeInCommand = new AlgaeIntakeInCommand(m_algaeIntakeSubsystem);
  public static final AlgaeIntakeOutCommand m_algaeIntakeOutCommand = new AlgaeIntakeOutCommand(m_algaeIntakeSubsystem);
  public static final ElevatorDownCommand m_elevatorFirstStageDownCommand = new ElevatorDownCommand(m_elevatorSubsystem);
  public static final ElevatorUpCommand m_elevatorFirstStageUpCommand = new ElevatorUpCommand(m_elevatorSubsystem);
  public static final Autos m_autos = new Autos(m_drivetrainSubsystem, m_objectTrackerSubsystem, m_elevatorSubsystem, m_algaeArmSubsystem, m_algaeIntakeSubsystem);
  public static final VisionAutoCommand m_visionAutoCommand = new VisionAutoCommand(m_drivetrainSubsystem, m_objectTrackerSubsystem); // TODO: Change this later
  public static final ClimberUpCommand m_climberUpCommand = new ClimberUpCommand(m_climberSubsystem);
  public static final ClimberDownCommand m_climberDownCommand = new ClimberDownCommand(m_climberSubsystem);
  public static final MoveClimbPos m_moveClimbPos = new MoveClimbPos(m_streamDeckSubsystem, m_algaeArmSubsystem, m_elevatorSubsystem);
  public static final MoveAlgaeToPose m_moveAlgaeToPose = new MoveAlgaeToPose(m_streamDeckSubsystem, m_algaeArmSubsystem, m_elevatorSubsystem);
  public static final AlgaeProcessorCommand m_algaeProcessorCommand = new AlgaeProcessorCommand(m_algaeIntakeSubsystem);
  public static final RunAutoCommand m_runAutoCommand = new RunAutoCommand(m_streamDeckSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // RIGHT BUTTONS
    Trigger algaeIntakeInButton = new JoystickButton(rightJoystick, Constants.ALGAE_INTAKE_IN_BUTTON);
    Trigger elevatorUpButton = new JoystickButton(rightJoystick, Constants.ELEVATOR_FIRST_STAGE_UP_BUTTON);
    Trigger elevatorDownButton = new JoystickButton(rightJoystick, Constants.ELEVATOR_FIRST_STAGE_DOWN_BUTTON);
    Trigger moveAlgaeToPose = new JoystickButton(rightJoystick, 4);
    // cancelTeleAuto button on VisionPureAutoCommand: rightJoystick, buttonNumber: 6
    Trigger resetButton = new JoystickButton(rightJoystick, Constants.SWERVE_RESET_BUTTON);
    

    // LEFT BUTTONS
    Trigger algaeIntakeOutButton = new JoystickButton(leftJoystick, Constants.ALGAE_INTAKE_OUT_BUTTON);
    Trigger runVisionAuto = new JoystickButton(rightJoystick, 2);
    Trigger tipCorrectionDisableButton = new JoystickButton(leftJoystick, Constants.TIP_CORRECTION_DISABLE_BUTTON);
    Trigger tipCorrectionEnableButton = new JoystickButton(leftJoystick, Constants.TIP_CORRECTION_ENABLE_BUTTON);
    Trigger climberUpButton = new JoystickButton(leftJoystick, Constants.CLIMB_UP_BUTTON);
    Trigger climberDownButton = new JoystickButton(leftJoystick, Constants.CLIMB_DOWN_BUTTON);
    Trigger algaeUpButton = new JoystickButton(leftJoystick,Constants.ALGAE_RAISE_ARM_BUTTON);
    Trigger algaeDownButton = new JoystickButton(leftJoystick, Constants.ALGAE_LOWER_ARM_BUTTON);

    //Trigger climberTestButton = new JoystickButton(leftJoystick, 5);
    
    Trigger climbPos = new JoystickButton(leftJoystick, 2);
    
    
    // Trigger zeroElevatorPowerButton = new JoystickButton(rightJoystick, Constants.ELEVATOR_ZERO_POWER_BUTTON); // If wanted to use choose a different, empty, button
    // Trigger processorButton = new JoystickButton(rightJoystick, Constants.PROCESSOR_OUT_BUTTON);
    // Trigger moveElevatorToPos = new JoystickButton(rightJoystick, 10);  

    //Trigger visionAutoData = new JoystickButton(leftJoystick, 11);

    // tipCorrectionDisableButton.onTrue(new InstantCommand(()->m_drivetrainSubsystem.setTriggerAntiTip(true)));
    // tipCorrectionDisableButton.onFalse(new InstantCommand(()->m_drivetrainSubsystem.setTriggerAntiTip(false)));
    // tipCorrectionEnableButton.onTrue(new InstantCommand(()->m_drivetrainSubsystem.setEnableAntiTip()));

    tipCorrectionDisableButton.onTrue(new InstantCommand(()->m_drivetrainSubsystem.setAntiTip(false)));
    tipCorrectionEnableButton.onTrue(new InstantCommand(()-> m_drivetrainSubsystem.setAntiTip(true)));

    // distancePidPathButton.onTrue(new VisionPureAutoCommand(m_drivetrainSubsystem, m_objectTrackerSubsystem, 0, 1, 0));
    // algaeIntakeInButton.onTrue(new InstantCommand(()->m_algaeArmSubsystem.moveArmUp()));
    // algaeIntakeOutButton.onTrue(new InstantCommand(()->m_algaeArmSubsystem.moveArmDown()));
    algaeIntakeInButton.whileTrue(m_algaeIntakeInCommand);
    algaeIntakeOutButton.whileTrue(m_algaeIntakeOutCommand);


    //climberTestButton.whileTrue(new InstantCommand(()->m_climberSubsystem.configure()));
    //climberTestButton.whileFalse(new InstantCommand(()->m_climberSubsystem.motor.setVoltage(0)));
    // climberUpButton.whileTrue(m_climberUpCommand);
    // climberDownButton.whileTrue(m_climberDownCommand);
    climberUpButton.whileTrue(new InstantCommand(()-> m_climberSubsystem.up()));
    climberUpButton.whileFalse(new InstantCommand(()->m_climberSubsystem.stop()));
    climberDownButton.whileTrue(new InstantCommand(()-> m_climberSubsystem.down()));
    climberDownButton.whileFalse(new InstantCommand(()->m_climberSubsystem.stop()));

    // Right Buttons, Run
    resetButton.onTrue(new SequentialCommandGroup(
      new InstantCommand(()-> m_drivetrainSubsystem.resetAngle()),
      new InstantCommand(()-> m_drivetrainSubsystem.zeroOdometry())
      ));
    // zeroElevatorPowerButton.onTrue(new InstantCommand(()-> m_elevatorSubsystem.zeroElevatorPower()));

    elevatorUpButton.onTrue(new InstantCommand(()-> m_elevatorSubsystem.upTargetPos(2000/3)));
    elevatorDownButton.onTrue(new InstantCommand(()-> m_elevatorSubsystem.downTargetPos(2000/3)));
    algaeUpButton.onTrue(new InstantCommand(()->m_algaeArmSubsystem.moveArmUp()));
    algaeDownButton.onTrue(new InstantCommand(()->m_algaeArmSubsystem.moveArmDown()));
    runVisionAuto.onTrue(m_runAutoCommand);
    // runVisionAuto.onTrue(m_autos.centerReef());
    // runVisionAuto.onTrue(m_autos.autoReefAndBarge());
    moveAlgaeToPose.onTrue(m_moveAlgaeToPose);

    // processorButton.whileTrue(m_algaeProcessorCommand); // ALGAE PROCESSOR OUT 


    // visionAutoData.onTrue(new InstantCommand(()->new VisionAutoCommand(m_drivetrainSubsystem, m_objectTrackerSubsystem, 8).visionAutoData(0.00001, -20, 0, 8)));


    climbPos.onTrue(m_moveClimbPos);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    String autoEntry = m_streamDeckSubsystem.getAutoEntry();
    switch (autoEntry) {
      case "C": //auto for picking up the algae in the middle and score barge
        return m_autos.centerReef();
      case "CS": // auto for one algae on the front and one algae on the side
        return m_autos.autoReefAndBarge(); //Default = hopefully Blue
      //  return m_autos.autoReefAndBargeRed(); //Comfirmed default doesn't work on Red
      case "SS": // auto for the two algae on the side of the reef, starts back left corner of bot on line, 6 ft from the wall, facing the reef
        return m_autos.autoReefAndBargeRight();
      default:
        return m_autos.autoReefAndBarge();
    } 
  }
}
