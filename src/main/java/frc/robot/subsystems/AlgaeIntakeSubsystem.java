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

public class AlgaeIntakeSubsystem extends SubsystemBase {
  /** Creates a new AlgaeIntakeSubsystem. */
  private final SparkMax m_leftAlgaeIntakeMotor;
  private final SparkMax m_rightAlgaeIntakeMotor;

  private final SparkMaxConfig m_leftAlgaeIntakeMotorConfig;
  private final SparkMaxConfig m_rightAlgaeIntakeMotorConfig;


  public AlgaeIntakeSubsystem() {
    m_leftAlgaeIntakeMotor = new SparkMax(Constants.ALGAE_LEFT_INTAKE_MOTOR, MotorType.kBrushless);
    m_rightAlgaeIntakeMotor = new SparkMax(Constants.ALGAE_RIGHT_INTAKE_MOTOR, MotorType.kBrushless);    

    m_leftAlgaeIntakeMotorConfig = new SparkMaxConfig();
    m_leftAlgaeIntakeMotorConfig.idleMode(IdleMode.kBrake);
    m_leftAlgaeIntakeMotorConfig.inverted(false);

    m_rightAlgaeIntakeMotorConfig = new SparkMaxConfig();
    m_rightAlgaeIntakeMotorConfig.idleMode(IdleMode.kBrake);
    m_rightAlgaeIntakeMotorConfig.inverted(false);

    m_leftAlgaeIntakeMotorConfig.follow(m_rightAlgaeIntakeMotor, true);

    m_leftAlgaeIntakeMotor.configure(
      m_leftAlgaeIntakeMotorConfig, 
      SparkBase.ResetMode.kResetSafeParameters, 
      SparkBase.PersistMode.kPersistParameters
    );
    m_rightAlgaeIntakeMotor.configure(
      m_rightAlgaeIntakeMotorConfig, 
      SparkBase.ResetMode.kResetSafeParameters, 
      SparkBase.PersistMode.kPersistParameters
    );

  }
  
  public void inAlgaeIntake() {
    m_rightAlgaeIntakeMotor.set(-Constants.ALGAE_INTAKE_ROTATION_SPEED);
  }

  public void outAlgaeIntake() {
    m_rightAlgaeIntakeMotor.set(Constants.ALGAE_INTAKE_ROTATION_SPEED);
  }

  public void stopAlgaeIntake() {
    m_leftAlgaeIntakeMotor.set(0);
    m_rightAlgaeIntakeMotor.set(0);
  }  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
