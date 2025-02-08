// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlgaeIntakeInCommand;
import frc.robot.commands.AlgaeIntakeOutCommand;
import frc.robot.commands.CoralIntakeInCommand;
import frc.robot.commands.CoralIntakeOutCommand;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  public static final AlgaeIntakeSubsystem m_algaeIntakeSubsystem = new AlgaeIntakeSubsystem();
  public static final CoralIntakeSubsystem m_coralIntakeSubsystem = new CoralIntakeSubsystem();

  // Commands
  public static final AlgaeIntakeInCommand m_algaeIntakeInCommand = new AlgaeIntakeInCommand(m_algaeIntakeSubsystem);
  public static final AlgaeIntakeOutCommand m_algaeIntakeOutCommand = new AlgaeIntakeOutCommand(m_algaeIntakeSubsystem);
  public static final CoralIntakeInCommand m_coralIntakeInCommand = new CoralIntakeInCommand(m_coralIntakeSubsystem);
  public static final CoralIntakeOutCommand m_coralIntakeOutCommand = new CoralIntakeOutCommand(m_coralIntakeSubsystem);

  // Joysticks
  public static Joystick rightJoystick = new Joystick(Constants.RIGHT_JOYSTICK_CHANNEL);
  public static Joystick leftJoystick = new Joystick(Constants.LEFT_JOYSTICK_CHANNEL);

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
    Trigger coralIntakeInButton = new JoystickButton(leftJoystick, Constants.CORAL_INTAKE_IN_BUTTON);
    Trigger coralIntakeOutButton = new JoystickButton(leftJoystick, Constants.CORAL_INTAKE_OUT_BUTTON);
    Trigger algaeIntakeInButton = new JoystickButton(leftJoystick, Constants.ALGAE_INTAKE_IN_BUTTON);
    Trigger algaeIntakeOutButton = new JoystickButton(leftJoystick, Constants.ALGAE_INTAKE_OUT_BUTTON);


    coralIntakeInButton.whileTrue(m_coralIntakeInCommand);
    coralIntakeOutButton.whileTrue(m_coralIntakeOutCommand);
    algaeIntakeInButton.whileTrue(m_algaeIntakeInCommand);
    algaeIntakeOutButton.whileTrue(m_algaeIntakeOutCommand);
    // Right Joystick Buttons, initialization
    Trigger resetButton = new JoystickButton(rightJoystick, Constants.SWERVE_RESET_BUTTON);

    // Right Buttons, Run
    resetButton.onTrue(new InstantCommand(()-> m_drivetrainSubsystem.zeroOdometry()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new Command() {
      
    };
  }
}
