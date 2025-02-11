// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  TalonFX m_climberMotor;
  public ClimberSubsystem() {
    m_climberMotor = new TalonFX(Constants.CLIMBER_MOTOR_ID);
    m_climberMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setClimberPower(double power){
    m_climberMotor.setVoltage(power * 11);
  }

  public void climberDown(){
    m_climberMotor.setVoltage(0); //TODO: Figure out this constant with testing maybe also have an adjustment system utilizing streamdeck
  }

  public void climberUp(){
    m_climberMotor.setVoltage(0); //TODO: Figure out this constant with testing maybe also have an adjustment system utilizing streamdeck
  }

  public void zeroClimber(){
    m_climberMotor.setVoltage(0);
  }

  @Override
  public void periodic() {
    //SmartDashboard.getNumber("Climber Temperature", m_climberMotor.getDeviceTemp().getUnits()); //its a string convert to int to get temperature 
    // This method will be called once per scheduler run
  }
}
