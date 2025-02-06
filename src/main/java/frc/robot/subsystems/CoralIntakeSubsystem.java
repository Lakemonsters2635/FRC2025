// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralIntakeSubsystem extends SubsystemBase {
  /** Creates a new CoralIntakeSubsystem. */
  private final SparkMax m_leftCoralIntakeMotor;
  private final SparkMax m_rightCoralIntakeMotor;

  private final SparkMaxConfig m_leftCoralIntakeMotorConfig;
  private final SparkMaxConfig m_rightCoralIntakeMotorConfig;


  public CoralIntakeSubsystem() {
    m_leftCoralIntakeMotor = new SparkMax(Constants.CORAL_LEFT_INTAKE_MOTOR, MotorType.kBrushless);
    m_rightCoralIntakeMotor = new SparkMax(Constants.CORAL_RIGHT_INTAKE_MOTOR, MotorType.kBrushless);    

    m_leftCoralIntakeMotorConfig = new SparkMaxConfig();
    m_leftCoralIntakeMotorConfig.idleMode(IdleMode.kBrake);
    m_leftCoralIntakeMotorConfig.inverted(false);
    

    m_rightCoralIntakeMotorConfig = new SparkMaxConfig();
    m_rightCoralIntakeMotorConfig.idleMode(IdleMode.kBrake);
    m_rightCoralIntakeMotorConfig.inverted(false);

    m_leftCoralIntakeMotorConfig.follow(m_rightCoralIntakeMotor, true);

    m_leftCoralIntakeMotor.configure(
      m_leftCoralIntakeMotorConfig, 
      SparkBase.ResetMode.kResetSafeParameters, 
      SparkBase.PersistMode.kPersistParameters
    );
    m_rightCoralIntakeMotor.configure(
      m_rightCoralIntakeMotorConfig, 
      SparkBase.ResetMode.kResetSafeParameters, 
      SparkBase.PersistMode.kPersistParameters
    );

  }
  
  public void inCoralIntake() {
    m_rightCoralIntakeMotor.set(Constants.CORAL_INTAKE_ROTATION_SPEED);
  }

  public void outCoralIntake() {
    m_rightCoralIntakeMotor.set(-Constants.CORAL_INTAKE_ROTATION_SPEED);
  }

  public void stopCoralIntake() {
    m_leftCoralIntakeMotor.set(0);
    m_rightCoralIntakeMotor.set(0);
  }  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
