// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeArmSubsystem extends SubsystemBase {
  private final SparkMax m_algaeArmMotor;
  private final SparkMaxConfig m_algaeArmConfig;
  double ff, fb, motorPower, theta, m_poseTarget = 25;
  double GAIN = - 0.412; // TODO: Measure this value
  double GAIN_ALGAE = - 0.78;
  PIDController m_algaeArmController;
  Joystick leftJoystick = new Joystick(0);
  double offset = 0;

  public AlgaeArmSubsystem() {
    m_algaeArmMotor = new SparkMax(Constants.ALGAE_ARM_MOTOR, SparkMax.MotorType.kBrushless);

    // Configuration
    m_algaeArmConfig = new SparkMaxConfig();
    m_algaeArmConfig.inverted(false);
    // m_algaeArmConfig.idleMode(IdleMode.kBrake);
    m_algaeArmConfig.idleMode(IdleMode.kCoast);
    m_algaeArmConfig
    .smartCurrentLimit(70);
    m_algaeArmMotor.configure(
      m_algaeArmConfig, 
      SparkBase.ResetMode.kNoResetSafeParameters, 
      SparkBase.PersistMode.kPersistParameters
    );

    m_algaeArmController = new PIDController(0.12,0,0); // TODO: Tune these values
    // m_algaeArmController = new PIDController(0.02,0,0); // TODO: Tune these values

    resetEncoder(); // Reset encoder, since we start from the 0 position
  }

  public void setArmPowerVolts(double volts) {
    m_algaeArmMotor.setVoltage(volts);
  }

  public double getEncoderCounts() {
    // getEncoderCounts() is in revolutions
    return m_algaeArmMotor.getEncoder().getPosition();
  }

  public double getDegrees() {
    // 160 is the gear ratio
    double degrees = (getEncoderCounts() /54) * 360;  //160 old gear ratio
    degrees += offset; // Calibration offset required with the setup
    degrees %= 360;
    return degrees;
  }

  public void resetEncoder() {
    m_algaeArmMotor.getEncoder().setPosition(0);
  }

  public void moveArmUp(){
    m_poseTarget -=5;
    // offset -=5;
    // setArmPosition(m_poseTarget-5);
  }
  public void moveArmDown(){
    m_poseTarget += 5;
    // offset +=5;
    // setArmPosition(m_poseTarget+5);
  }

  public double controlArmThrottle() {
    
    
    return leftJoystick.getThrottle() * 0.1 * 12;
  }

  public void setArmPosition(double position) {
    m_poseTarget = position;
  }

  @Override
  public void periodic() {
    theta = getDegrees();

    // setArmPowerVolts(controlArmThrottle());
    m_poseTarget = MathUtil.clamp(m_poseTarget, 10, 150);
    ff = GAIN_ALGAE * Math.abs(Math.sin(Math.toRadians(theta)));
    fb = MathUtil.clamp(m_algaeArmController.calculate(theta, m_poseTarget), -6, 6);
    // fb = MathUtil.clamp(m_algaeArmController.calculate(theta, m_poseTarget), -6, 6);

    SmartDashboard.putNumber("algae m_poseTarget", m_poseTarget);
    SmartDashboard.putNumber("Encoder Counts", getEncoderCounts());
    SmartDashboard.putNumber("Algae Degrees", getDegrees());
    SmartDashboard.putNumber("Motor Power", m_algaeArmMotor.get());
    SmartDashboard.putNumber("Motor Power Volts", m_algaeArmMotor.get()*11);
    SmartDashboard.putNumber("Feed Forward", ff);
    SmartDashboard.putNumber("Feed Back", fb);
    SmartDashboard.putNumber("Algae Throttle Motor Power", leftJoystick.getThrottle() * 0.2 * 12);
    SmartDashboard.putNumber("Algae AppliedVoltage", m_algaeArmMotor.getAppliedOutput());
    SmartDashboard.putNumber("Algae CurrentOutput", m_algaeArmMotor.getOutputCurrent());

    // setArmPosition(-Math.abs(leftJoystick.getThrottle()*90));
    SmartDashboard.putNumber("Throttle angle", -Math.abs(leftJoystick.getThrottle()*90));
    SmartDashboard.putNumber("ff + fb", fb + ff);
    setArmPowerVolts(ff + fb);
    // setArmPowerVolts(ff + controlArmThrottle());
    
  }
}
