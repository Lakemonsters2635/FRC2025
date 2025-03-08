// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  public TalonFX motor;
  Joystick leftJoystick = new Joystick(0);
  public ClimberSubsystem() {
    motor = new TalonFX(8, "CANivore");
  }

  public void up(){
    motor.setVoltage(2.0);
  }

  public void down(){
    // motor.setVoltage((leftJoystick.getThrottle() - 1) *3);
    motor.setVoltage(-1.125 + (leftJoystick.getThrottle()-1));
  }

  public void stop(){
    motor.setVoltage(0);
  }

  public void hold(){
    motor.setVoltage(0);
  }

  public void configure(){
    motor.setVoltage(leftJoystick.getThrottle() * 2);
    SmartDashboard.putNumber("Throttle", leftJoystick.getThrottle()* 2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // motor.setVoltage(leftJoystick.getThrottle()* 2.5);
    // SmartDashboard.putNumber("Throttle", leftJoystick.getThrottle()* 2.5);
    SmartDashboard.putNumber("ClimberEncoder", motor.getPosition().getValueAsDouble());
  }
}

