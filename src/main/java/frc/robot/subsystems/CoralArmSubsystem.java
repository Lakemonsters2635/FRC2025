// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
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
  private final double GAIN_CORRAL = 0.416 * 1.6; // Holds with corral in horizontal position
  private final double GAIN_WITHOUT_CORRAL = 0.1 *1.6;
  double gain = GAIN_CORRAL;
  PIDController m_coralArmController;
  double theta;
  double m_poseTarget;

  public CoralArmSubsystem() {
    m_coralArmMotor = new SparkMax(Constants.CORAL_ARM_MOTOR, MotorType.kBrushless); 
    m_coralArmMotorConfig = new SparkMaxConfig();
    m_coralArmMotorConfig.idleMode(IdleMode.kBrake);
    m_coralArmController = new PIDController(.1, 0, 0);
    m_coralArmMotorConfig.smartCurrentLimit(10);
    m_coralArmMotor.getEncoder().setPosition(0); // reset encoder
    m_coralArmMotor.configure(m_coralArmMotorConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  public void setArmPower(double armPower){
    m_coralArmMotor.setVoltage(armPower);
  }

  public void stopArmPower(){
    m_coralArmMotor.setVoltage(0);
  }

  public double getEncoderCounts() {
    // getEncoderCounts() is in revolutions
    return m_coralArmMotor.getEncoder().getPosition();
  }

  public double getDegrees() {
    // encoder offset should be zero since the relative encoder is resetted and setup correctly before deploy
    // 100 is the gear ratio
    double degrees = ((getEncoderCounts()-Constants.CORAL_ARM_ENCODER_OFFSET)/100)*360;
    degrees%=360;
    return degrees;
  }

  public void resetEncoderPosition() {
    m_coralArmMotor.getEncoder().setPosition(0);
  }

  public double controlArmThrottle() {
    Joystick leftJoystick = new Joystick(0);
    SmartDashboard.putNumber("Throttle Motor Power", leftJoystick.getThrottle()*0.1*12);
    return leftJoystick.getThrottle() * 0.1 * 12;
  }

  public boolean isAtTarget(){
    if (Math.abs(getDegrees()-m_poseTarget)< 4) {
      return true;
    }

    return false;
  }

  public void setPoseTarget(double poseTarget) {
    m_poseTarget = poseTarget;
  }

  @Override
  public void periodic() {
    theta = getDegrees();

    ff = gain*Math.abs(Math.sin(Math.toRadians(getDegrees()))); // Getdegrees is negative as the corral arm goes down from initial position
    fb = MathUtil.clamp(m_coralArmController.calculate(theta, m_poseTarget), -2, 2);
    SmartDashboard.putNumber("coralArm Encoder Counts", getEncoderCounts());
    SmartDashboard.putNumber("coralArm Degrees", getDegrees());
    SmartDashboard.putNumber("coralArm Motor Power Volts", m_coralArmMotor.getAppliedOutput());
    SmartDashboard.putNumber("coralArm Feed Forward", ff);
    SmartDashboard.putNumber("coralArm Feed Back", fb);

    //setArmPower(ff);
    setArmPower(ff+fb);

    
  }
}