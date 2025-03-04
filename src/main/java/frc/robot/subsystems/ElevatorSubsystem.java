// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  public static final Encoder m_encoderInner = new Encoder(Constants.INNER_ELEVATOR_ENCODER_A, Constants.INNER_ELEVATOR_ENCODER_B);
  public static final Encoder m_encoderOuter = new Encoder(Constants.OUTER_ELEVATOR_ENCODER_A, Constants.OUTER_ELEVATOR_ENCODER_B);

  double fb, m_poseTarget;
  double ff = -.5;
  PIDController m_elevatorController;

  public boolean isPIDControl = true;

  Joystick leftJoystick = new Joystick(0);
  // Trigger stopElevator = new JoystickButton(leftJoystick, 8);

  

  TalonFX m_elevatorMotor;

  public Command unspoolElevator = new SequentialCommandGroup(
    new WaitCommand(1),
    new InstantCommand(()->m_elevatorMotor.setVoltage(0.5)),
    new WaitCommand(1),
    new InstantCommand(()->m_elevatorMotor.setVoltage(0))
  );

  public ElevatorSubsystem() {
    m_elevatorMotor = new TalonFX(Constants.ElEVATOR_MOTOR_ID, new CANBus("CANivore"));
    m_elevatorMotor.setNeutralMode(NeutralModeValue.Brake);

    m_elevatorController = new PIDController(0.001, 0, 0);

    
    // m_elevatorController = new PIDController(0.001, 0, 0); 

    m_encoderInner.reset();
    m_encoderOuter.reset();
  }
  //negative up
  //positive down

  public void setElevatorMotorPower(double power){
    // m_elevatorMotor.setVoltage(power * 11); //11 volts
    m_elevatorMotor.setVoltage(power);
  }

  public double innerEncoderRotations(){
    return -m_encoderInner.get();
  }
  
  public double outerEncoderRotations(){
    return -m_encoderOuter.get();
  }


  public double elevatorHeight() {
    // m_elevatorMotor
    // return (36000 * m_elevatorMotor.getPosition().getValueAsDouble())/(-92); this is for the embedded encoder
    return (innerEncoderRotations() + outerEncoderRotations()) * (24/16);
  }

  public void setRaisePower(){
    m_elevatorMotor.setVoltage(-3);
  }

  public void setStageHoldPower(){
    m_elevatorMotor.setVoltage(-0.85);
  }

  public void setFirstStageLowerPower(){
    m_elevatorMotor.setVoltage(1);
  }

  public void upTargetPos(double increase){
    m_poseTarget+= increase;
  }

  public void downTargetPos(double decrease){
    m_poseTarget-= decrease;
  }

  public void zeroElevatorPower(){
    m_elevatorMotor.setVoltage(0);
  }

  public void setElevatorTarget(double targetPos){
    m_poseTarget = targetPos;
  }

  public boolean isAtPosition(){
    if (Math.abs(elevatorHeight()-m_poseTarget) < 1000) {
      return true;
    }

    return false;
  }

  @Override
  public void periodic() {
    m_poseTarget = MathUtil.clamp(m_poseTarget, -9000, 32500);
    // This method will be called once per schedu
    // SmartDashboard.putNumber("innerEncoder Rot", innerEncoderRotations());
    // SmartDashboard.putNumber("outerEncoder Rot", outerEncoderRotations());
    SmartDashboard.putNumber("elevatorHeight", elevatorHeight());
    SmartDashboard.putNumber("Motor Power (-1 to 1)", m_elevatorMotor.get());
    SmartDashboard.putNumber("Motor Power Voltage", m_elevatorMotor.get() * 11);

    // m_poseTarget = leftJoystick.getThrottle()*10000;
    
    fb = -1 * m_elevatorController.calculate(elevatorHeight(), m_poseTarget);

    // stopElevator.toggleOnFalse(new InstantCommand(()->setElevatorMotorPower(MathUtil.clamp(ff+fb, -3, 1.5))));
    // stopElevator.toggleOnTrue(new InstantCommand(()->setStageHoldPower()));

    // setElevatorMotorPower(MathUtil.clamp(ff+fb, -3, 1.5));
    // setElevatorMotorPower(MathUtil.clamp(ff+fb, -4.5, 1.5));

    SmartDashboard.putNumber("ElevatorVolts", m_elevatorMotor.getMotorVoltage().getValueAsDouble());

    // if (stopElevator.getAsBoolean()) {
    //   isPIDControl = false;
      
    // }

    if (isPIDControl) {
      setElevatorMotorPower(MathUtil.clamp(ff+fb, -5.5, 4));
    }

    // if () {
    //   setElevatorMotorPower(MathUtil.clamp(ff+fb, -3, 1.5));
    // }
    // else{
    //   setStageHoldPower();
    // }
    // if (Math.abs(elevatorHeight() - m_poseTarget) > 1000) {
    //   setElevatorMotorPower(MathUtil.clamp(ff+fb, -3, 1.5));
    // }
    // else{
    //   setStageHoldPower();
    // }

    // if(elevatorHeight() > 34000){
    //   setElevatorMotorPower(0.5);
    // }
    // if(elevatorHeight() < -12000){
    //   setElevatorMotorPower(-1);;
    // }

    // setElevatorMotorPower(ff);
  }
}
