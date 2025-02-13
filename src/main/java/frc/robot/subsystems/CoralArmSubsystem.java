// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralArmSubsystem extends SubsystemBase {
  /** Creates a new CoralArmSubsystem. */
  private final SparkMax m_coralArmMotor;
  private final SparkMaxConfig m_coralArmMotorConfig;
  double ff;
  double fb;
  double motorPower;
  double gain = 0.052755904197693;
  PIDController m_coralArmController;
  double theta;
  double m_poseTarget;

  public CoralArmSubsystem() {
    m_coralArmMotor = new SparkMax(1, MotorType.kBrushless); 
    m_coralArmMotorConfig = new SparkMaxConfig();
    m_coralArmMotorConfig.idleMode(IdleMode.kBrake);
    m_coralArmController = new PIDController(0, 0, 0);
    m_coralArmMotorConfig.smartCurrentLimit(10);
    m_coralArmMotor.getEncoder().setPosition(0); // reset encoder
    m_coralArmMotor.configure(m_coralArmMotorConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  public void setArmPower(double armPower){
    m_coralArmMotor.setVoltage(armPower * 12);
  }

  public void stopArmPower(){
    m_coralArmMotor.setVoltage(0);
  }

  public double getEncoderCounts() {
    return m_coralArmMotor.getEncoder().getPosition();
  }

  public double getDegrees() {
    // encoder offset should be zero since the relative encoder is resetted and setup correctly before deploy
    double degrees = ((getEncoderCounts()-Constants.CORAL_ARM_ENCODER_OFFSET)/160)*360;
    degrees%=360;
    return degrees;
  }

  public void resetEncoderPosition() {
    m_coralArmMotor.getEncoder().setPosition(0);
  }

  public void controlArmThrottle() {
    Joystick leftJoystick = new Joystick(0);
    m_coralArmMotor.setVoltage(leftJoystick.getThrottle()*0.1*12);
    SmartDashboard.putNumber("Throttle Motor Power", leftJoystick.getThrottle() * 0.3);
  }

  public void setPoseTarget(double poseTarget) {
    m_poseTarget = poseTarget;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    theta = getDegrees();
    controlArmThrottle();
    ff = gain*Math.sin(Math.toRadians(getDegrees()));
    //setArmPower(ff);
    fb = m_coralArmController.calculate(theta, m_poseTarget);
    SmartDashboard.putNumber("Encoder Counts", getEncoderCounts());
    SmartDashboard.putNumber("Degrees", getDegrees());
    SmartDashboard.putNumber("Motor Power", m_coralArmMotor.get());
    SmartDashboard.putNumber("Motor Power Volts", m_coralArmMotor.get()*11);
    SmartDashboard.putNumber("Feed Forward", ff);
    SmartDashboard.putNumber("Feed Back", fb);
  }
}