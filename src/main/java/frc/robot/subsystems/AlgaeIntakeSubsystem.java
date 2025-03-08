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
    m_leftAlgaeIntakeMotorConfig.smartCurrentLimit(10);

    m_rightAlgaeIntakeMotorConfig = new SparkMaxConfig();
    m_rightAlgaeIntakeMotorConfig.idleMode(IdleMode.kBrake);
    m_rightAlgaeIntakeMotorConfig.inverted(false);
    m_rightAlgaeIntakeMotorConfig.smartCurrentLimit(10);

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

  public void holdAlgaeIntake() {
    m_rightAlgaeIntakeMotor.setVoltage(Constants.ALGAE_INTAKE_VOLTAGE_HOLD);
    // m_leftAlgaeIntakeMotor.setVoltage(-Constants.ALGAE_INTAKE_VOLTAGE_HOLD);
  }
  
  public void inAlgaeIntake() {
    m_rightAlgaeIntakeMotor.setVoltage(Constants.ALGAE_INTAKE_VOLTAGE_IN);
    // m_leftAlgaeIntakeMotor.setVoltage(-Constants.ALGAE_INTAKE_VOLTAGE_IN);
  }

  public void outAlgaeIntake() {
    m_rightAlgaeIntakeMotor.setVoltage(Constants.ALGAE_INTAKE_VOLTAGE_OUT);
    // m_leftAlgaeIntakeMotor.setVoltage(-Constants.ALGAE_INTAKE_VOLTAGE_OUT);
  }

  public void stopAlgaeIntake() {
    m_rightAlgaeIntakeMotor.setVoltage(0);
    // m_leftAlgaeIntakeMotor.setVoltage(0);
  }  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
