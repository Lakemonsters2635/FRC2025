// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlgaeIntakeInCommand;
import frc.robot.commands.AlgaeIntakeOutCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.ClimberDownCommand;
import frc.robot.commands.ClimberUpCommand;
import frc.robot.commands.CoralIntakeInCommand;
import frc.robot.commands.CoralIntakeOutCommand;
import frc.robot.commands.ElevatorDownCommand;
import frc.robot.commands.ElevatorUpCommand;
import frc.robot.commands.MoveAlgaeToPose;
import frc.robot.commands.MoveClimbPos;
import frc.robot.commands.MoveCoralArmToPosition;
import frc.robot.commands.MoveElevatorToPoseCommand;
import frc.robot.commands.RunAutoCommand;
import frc.robot.commands.VisionAutoCommand;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralArmSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
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
  public static final CoralIntakeSubsystem m_coralIntakeSubsystem = new CoralIntakeSubsystem();
  public static final AlgaeArmSubsystem m_algaeArmSubsystem = new AlgaeArmSubsystem();
  public static final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  public static final CoralArmSubsystem m_coralArmSubsystem = new CoralArmSubsystem();
  public static final StreamDeckSubsystem m_streamDeckSubsystem = new StreamDeckSubsystem();
  public static final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  public static final ObjectTrackerSubsystem m_objectTrackerSubsystem = new ObjectTrackerSubsystem("front");

  // Commands
  public static final AlgaeIntakeInCommand m_algaeIntakeInCommand = new AlgaeIntakeInCommand(m_algaeIntakeSubsystem);
  public static final AlgaeIntakeOutCommand m_algaeIntakeOutCommand = new AlgaeIntakeOutCommand(m_algaeIntakeSubsystem);
  public static final CoralIntakeInCommand m_coralIntakeInCommand = new CoralIntakeInCommand(m_coralIntakeSubsystem);
  public static final CoralIntakeOutCommand m_coralIntakeOutCommand = new CoralIntakeOutCommand(m_coralIntakeSubsystem);
  public static final ElevatorDownCommand m_elevatorFirstStageDownCommand = new ElevatorDownCommand(m_elevatorSubsystem);
  public static final ElevatorUpCommand m_elevatorFirstStageUpCommand = new ElevatorUpCommand(m_elevatorSubsystem);
  public static final MoveCoralArmToPosition m_moveCoralArmToPosition = new MoveCoralArmToPosition(m_coralArmSubsystem, m_streamDeckSubsystem, m_algaeArmSubsystem, m_elevatorSubsystem);
  public static final MoveAlgaeToPose m_moveAlgaeToPose = new MoveAlgaeToPose(m_coralArmSubsystem, m_streamDeckSubsystem, m_algaeArmSubsystem, m_elevatorSubsystem);
  public static final Autos m_autos = new Autos(m_drivetrainSubsystem, m_objectTrackerSubsystem, m_elevatorSubsystem, m_coralArmSubsystem, m_algaeArmSubsystem);
  public static final VisionAutoCommand m_visionAutoCommand = new VisionAutoCommand(m_drivetrainSubsystem, m_objectTrackerSubsystem); // TODO: Change this later
  public static final ClimberUpCommand m_climberUpCommand = new ClimberUpCommand(m_climberSubsystem);
  public static final ClimberDownCommand m_climberDownCommand = new ClimberDownCommand(m_climberSubsystem);
  public static final MoveClimbPos m_moveClimbPos = new MoveClimbPos(m_coralArmSubsystem, m_streamDeckSubsystem, m_algaeArmSubsystem, m_elevatorSubsystem);

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
    //LEFT BUTTONS
    Trigger coralIntakeOutButton = new JoystickButton(leftJoystick, Constants.CORAL_INTAKE_OUT_BUTTON);
    Trigger algaeIntakeInButton = new JoystickButton(leftJoystick, Constants.ALGAE_INTAKE_IN_BUTTON);
    Trigger algaeIntakeOutButton = new JoystickButton(leftJoystick, Constants.ALGAE_INTAKE_OUT_BUTTON);
    Trigger tipCorrectionTriggerButton = new JoystickButton(leftJoystick, Constants.TIP_CORRECTION_TRIGGER_BUTTON);
    Trigger tipCorrectionEnableButton = new JoystickButton(leftJoystick, Constants.TIP_CORRECTION_ENABLE_BUTTON);
    Trigger setDriveSpeed = new JoystickButton(leftJoystick, 10);



    //Trigger climberTestButton = new JoystickButton(leftJoystick, 5);
    Trigger climberUpButton = new JoystickButton(leftJoystick, 5);
    Trigger climberDownButton = new JoystickButton(leftJoystick, 3);
    Trigger climbPos = new JoystickButton(leftJoystick, 2);
    //RIGHT BUTTONS
    Trigger elevatorUpButton = new JoystickButton(rightJoystick, Constants.ELEVATOR_FIRST_STAGE_UP_BUTTON);
    Trigger elevatorDownButton = new JoystickButton(rightJoystick, Constants.ELEVATOR_FIRST_STAGE_DOWN_BUTTON);
    Trigger resetButton = new JoystickButton(rightJoystick, Constants.SWERVE_RESET_BUTTON);
    Trigger zeroElevatorPowerButton = new JoystickButton(rightJoystick, Constants.ELEVATOR_ZERO_POWER_BUTTON);
    Trigger coralIntakeInButton = new JoystickButton(rightJoystick, Constants.CORAL_INTAKE_IN_BUTTON);
    Trigger moveCoralArmPos = new JoystickButton(rightJoystick, 6);
    Trigger moveAlgaeToPose = new JoystickButton(rightJoystick, 4);
    // Trigger moveElevatorToPos = new JoystickButton(rightJoystick, 10);  
    Trigger algaeUpButton = new JoystickButton(rightJoystick, 12);
    Trigger algaeDownButton = new JoystickButton(rightJoystick, 10);
    Trigger runVisionAuto = new JoystickButton(rightJoystick, 2);

    //Trigger visionAutoData = new JoystickButton(leftJoystick, 11);
    Trigger coralUpButton = new JoystickButton(leftJoystick, 11);
    Trigger coralDownButton = new JoystickButton(leftJoystick, 9);

    tipCorrectionTriggerButton.onTrue(new InstantCommand(()->m_drivetrainSubsystem.setTriggerAntiTip(true)));
    tipCorrectionTriggerButton.onFalse(new InstantCommand(()->m_drivetrainSubsystem.setTriggerAntiTip(false)));
    tipCorrectionEnableButton.onTrue(new InstantCommand(()->m_drivetrainSubsystem.setEnableAntiTip()));
    setDriveSpeed.onTrue(new InstantCommand(()->m_drivetrainSubsystem.setDriveSpeed(0.5)));

    coralUpButton.onTrue(new InstantCommand(()->m_coralArmSubsystem.moveCorralDown()));
    coralDownButton.onTrue(new InstantCommand(()->m_coralArmSubsystem.moveCorralUp()));
    coralIntakeInButton.whileTrue(m_coralIntakeInCommand);
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
      new InstantCommand(()-> m_drivetrainSubsystem.zeroOdometry()),
      new InstantCommand(()-> m_drivetrainSubsystem.resetAngle())));
    zeroElevatorPowerButton.onTrue(new InstantCommand(()-> m_elevatorSubsystem.zeroElevatorPower()));
    moveCoralArmPos.onTrue(m_moveCoralArmToPosition);
    // elevatorUpButton.whileTrue(m_elevatorFirstStageUpCommand);
    // elevatorDownButton.whileTrue(m_elevatorFirstStageDownCommand);
    elevatorUpButton.onTrue(new InstantCommand(()-> m_elevatorSubsystem.upTargetPos(2000/3)));
    elevatorDownButton.onTrue(new InstantCommand(()-> m_elevatorSubsystem.downTargetPos(2000/3)));
    coralIntakeOutButton.whileTrue(m_coralIntakeOutCommand);
    // moveElevatorToPos.onTrue(new MoveElevatorToPoseCommand(m_elevatorSubsystem, 20000));
    moveAlgaeToPose.onTrue(m_moveAlgaeToPose);
    algaeUpButton.onTrue(new InstantCommand(()->m_algaeArmSubsystem.moveArmUp()));
    algaeDownButton.onTrue(new InstantCommand(()->m_algaeArmSubsystem.moveArmDown()));
    runVisionAuto.onTrue(new RunAutoCommand(m_streamDeckSubsystem));

    // visionAutoData.onTrue(new InstantCommand(()->new VisionAutoCommand(m_drivetrainSubsystem, m_objectTrackerSubsystem, 8).visionAutoData(0.00001, -20, 0, 8)));


    climbPos.onTrue(m_moveClimbPos);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return m_autos.goStraight();
    return m_autos.goStraight();
  }
}
