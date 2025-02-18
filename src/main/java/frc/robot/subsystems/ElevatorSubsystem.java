// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  public static final Encoder m_encoderInner = new Encoder(Constants.INNER_ELEVATOR_ENCODER_A, Constants.INNER_ELEVATOR_ENCODER_B);
  public static final Encoder m_encoderOuter = new Encoder(Constants.OUTER_ELEVATOR_ENCODER_A, Constants.OUTER_ELEVATOR_ENCODER_B);

  TalonFX m_elevatorMotor;

  public ElevatorSubsystem() {
    m_elevatorMotor = new TalonFX(Constants.ElEVATOR_MOTOR_ID, new CANBus("CANivore"));
    m_elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
  }
  //negative up
  //positive down

  public void setElevatorMotorPower(double power){
    // m_elevatorMotor.setVoltage(power * 11); //11 volts
    m_elevatorMotor.setVoltage(power);
  }

  public double innerEncoderRotations(){
    return m_encoderInner.get();
  }
  
  public double outerEncoderRotations(){
    return m_encoderOuter.get();
  }

  public void setRaisePower(){
    m_elevatorMotor.setVoltage(-1.8);
  }

  public void setStageHoldPower(){
    m_elevatorMotor.setVoltage(-.85);
  }

  public void setFirstStageLowerPower(){
    m_elevatorMotor.setVoltage(1);
  }


  public void zeroElevatorPower(){
    m_elevatorMotor.setVoltage(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per schedu
    SmartDashboard.putNumber("innerEncoder Rot", innerEncoderRotations());
    SmartDashboard.putNumber("outerEncoder Rot", outerEncoderRotations());
    SmartDashboard.putNumber("Motor Power (-1 to 1)", m_elevatorMotor.get());
    SmartDashboard.putNumber("Motor Power Voltage", m_elevatorMotor.get() * 11);


  }
}
