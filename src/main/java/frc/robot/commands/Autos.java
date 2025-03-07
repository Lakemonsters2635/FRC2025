// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.CoralArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ObjectTrackerSubsystem;

/** Add your docs here. */
public class Autos {
    DrivetrainSubsystem m_dts;
    ObjectTrackerSubsystem m_ots;
    ElevatorSubsystem m_es;
    CoralArmSubsystem m_cas;
    AlgaeArmSubsystem m_aas;
    public Autos(DrivetrainSubsystem dts, ObjectTrackerSubsystem ots, ElevatorSubsystem es, CoralArmSubsystem cas, AlgaeArmSubsystem aas) {
        m_dts = dts;
        m_ots = ots;
        m_es = es;
        m_cas = cas;
        m_aas = aas;
    }

    public Command goStraight(){
        return new SequentialCommandGroup(
            new InstantCommand(()-> m_dts.stopMotors()),
            new InstantCommand(()->m_dts.setFollowJoystick(false)),
            new InstantCommand(()->m_es.setElevatorTarget(Constants.E_STATE_AUTO_FINAL.ELEVATOR_POSITION)),
            new InstantCommand(()->m_cas.setPoseTarget(Constants.E_STATE_AUTO_FINAL.CORAL_ARM_ANGLE)),
            new InstantCommand(()->m_aas.setArmPosition(Constants.E_STATE_AUTO_FINAL.ALGAE_ARM_ANGLE)),
            m_dts.createPath(
                new Pose2d(0,0, new Rotation2d(Units.degreesToRadians(-90))), 
                new Translation2d(0, 0.5), 
                new Pose2d(0, 1, new Rotation2d(Units.degreesToRadians(-90)))
            ),
            new InstantCommand(()->m_dts.setFollowJoystick(true)),

            // m_dts.createPath(
            //     new Pose2d(0,2.8, new Rotation2d(Units.degreesToRadians(90))), 
            //     new Translation2d(0, 2.9), 
            //     new Pose2d(0, 3, new Rotation2d(Units.degreesToRadians(90)))
            // ),
            new InstantCommand(()->m_dts.stopMotors()),
            new InstantCommand(()->m_dts.resetAngle(180)),
            new InstantCommand(()->m_dts.zeroOdometry()))
            ;
    }

    public Command visionAuto() {
        return new SequentialCommandGroup(
            new VisionAutoCommand(m_dts, m_ots)
        );
    } 


    // Source line-up auto
    public Command sourceLineup(){
        return new SequentialCommandGroup(
            // new VisionAutoCommand(m_dts, m_ots, Constants.SOURCE_TAG_IDS, 0.0, 0.0, 0.0) // TODO: determine these values
            new VisionAutoCommand(m_dts, m_ots, -1, 0.0, -1*(5+30), 0.0) // TODO: determine these values
            // new VisionAutoCommand(m_dts, m_ots, Constants.SOURCE_TAG_IDS, 0.0, -1*(5+30), 0.0) // TODO: determine these values
        );
    }

    // Corral line-up left
    public Command reefCorralLeft(){
        return new SequentialCommandGroup(
            new VisionAutoCommand(m_dts, m_ots, -1, 13, -1*(20+11), 0.0)
        );
    }

    // Corral line-up right
    public Command reefCorralRight(){
        return new SequentialCommandGroup(
            new VisionAutoCommand(m_dts, m_ots, -1, -13, -1*(20+11), 0) // TODO
        );
    }

    // Corral line-up algae
    public Command reefAlgae(){
        return new SequentialCommandGroup(
            // new InstantCommand(()-> m_dts.stopMotors()),
            // new WaitCommand(0.5),
            // new InstantCommand(()-> SmartDashboard.putString("reefAlgae", "runs")),
            // new VisionAutoCommand(m_dts, m_ots, 8)
            new VisionAutoCommand(m_dts, m_ots, -1, 0, -1 *(20 + 12), 0)
        );
    }
    // public Command reefAlgae(){
    //     return new SequentialCommandGroup(
    //         new VisionAutoCommand(m_dts, m_ots, Constants.REEF_TAG_IDS, 0, 48, 0)
    //     );
    // }

    // Line-up processor
    public Command processorAlgae(){
        return new SequentialCommandGroup(
            new VisionAutoCommand(m_dts, m_ots, -1, 0, 0, 0)
        );
    }

    public Command closestAprilTag(){
        return new SequentialCommandGroup(
            new VisionAutoCommand(m_dts, m_ots, -1, 0, -30, 0)
        );
    }
}
