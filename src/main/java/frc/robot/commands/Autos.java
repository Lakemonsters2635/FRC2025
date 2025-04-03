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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ObjectTrackerSubsystem;

/** Add your docs here. */
public class Autos {
    DrivetrainSubsystem m_dts;
    ObjectTrackerSubsystem m_ots;
    ElevatorSubsystem m_es;
    AlgaeArmSubsystem m_aas;
    AlgaeIntakeSubsystem m_ais;
    public Autos(DrivetrainSubsystem dts, ObjectTrackerSubsystem ots, ElevatorSubsystem es, AlgaeArmSubsystem aas, AlgaeIntakeSubsystem ais) {
        m_dts = dts;
        m_ots = ots;
        m_es = es;
        m_aas = aas;
        m_ais = ais;
    }

    public Command goStraight(){
        return new SequentialCommandGroup(
            new InstantCommand(()-> m_dts.stopMotors()),
            new InstantCommand(()->m_dts.setFollowJoystick(false)),
            new InstantCommand(()->m_es.setElevatorTarget(Constants.E_STATE_AUTO_FINAL.ELEVATOR_POSITION)),
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
            new VisionAutoCommand(m_dts, m_ots, -1, 0, -1 *(20 + 3), 0)
        );
    }
    // public Command reefAlgae(){
    //     return new SequentialCommandGroup(
    //         // new InstantCommand(()-> m_dts.stopMotors()),
    //         // new WaitCommand(0.5),
    //         // new InstantCommand(()-> SmartDashboard.putString("reefAlgae", "runs")),
    //         // new VisionAutoCommand(m_dts, m_ots, 8)
    //         new VisionAutoCommand(m_dts, m_ots, -1, 0, -1 *(20 + 12), 0)
    //     );
    // }
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

    public Command bargeAlgae(){
        return new SequentialCommandGroup(
            new VisionPureAutoCommand(m_dts, m_ots, 3),
            new WaitCommand(0.2)
            // TODO: work on this
        );
    }
    public Command multipleVisionAutos(){
        return new SequentialCommandGroup(
            new VisionPureAutoCommand(m_dts, m_ots, 3),
            new WaitCommand(2),
            new InstantCommand(()->m_dts.resetOdometry(new Pose2d(0,0, m_dts.getPose().getRotation()))),
            // We need to reset odometry before we do a createPath but do not want to reset the angle
            m_dts.createPath(
                new Pose2d(0,0, new Rotation2d(-90)), 
                new Translation2d(0,-0.75), 
                new Pose2d(0,-1.5, new Rotation2d(-90))),
            new WaitCommand(2),
            new VisionPureAutoCommand(m_dts, m_ots, 8)
        );
    }
    public Command chainPureMultiVision(){
        return new SequentialCommandGroup(
            new VisionPureAutoCommand(m_dts, m_ots, 8),
            new WaitCommand(.2),
            new VisionPureAutoCommand(m_dts, m_ots, 8, 0,-90, 70),
            new WaitCommand(.2),
            new VisionPureAutoCommand(m_dts, m_ots, 14),
            new WaitCommand(.2),
            new VisionPureAutoCommand(m_dts, m_ots, 14, -10, -90, -80),
            new WaitCommand(.2),
            new VisionPureAutoCommand(m_dts, m_ots, 8)
        );
    }

    public Command autoReefAndBarge(){
        return new SequentialCommandGroup(
            new MoveElevatorAndALgae(m_aas, m_es, Constants.E_STATE_ALGAE_LOW).withTimeout(1),
            new InstantCommand(()->m_ais.inAlgaeIntake()).withTimeout(0.2),
            // new ParallelCommandGroup(
                new VisionPureAutoCommand(m_dts, m_ots, 8, 0, (-1 *(20 + 3)) - 8, 0),
            //     new InstantCommand(()->m_ais.inAlgaeIntake()).withTimeout(10)
            // ),
            new WaitCommand(0.3),
            new InstantCommand(()->m_ais.holdAlgaeIntake()).withTimeout(1),

            new WaitCommand(2),
            new VisionPureAutoCommand(m_dts, m_ots, 0, -26 / Constants.INCHES_PER_METER, 0),
            new MoveElevatorAndALgae(m_aas, m_es, new Constants.ElevatorState(16395-1500+9000-((2000/3) * 11), 0, 20)).withTimeout(0.1),
            new VisionPureAutoCommand(m_dts, m_ots, 40 / Constants.INCHES_PER_METER, (-24-8) / Constants.INCHES_PER_METER, -160),
            new MoveElevatorAndALgae(m_aas, m_es, Constants.E_STATE_ALGAE_BARGE).withTimeout(2),
            new WaitCommand(0.2),
            new InstantCommand(()->m_ais.outAlgaeIntake()).withTimeout(1),
            new WaitCommand(1),
            new InstantCommand(()->m_ais.stopAlgaeIntake()).withTimeout(0.1),
            new WaitCommand(0.5), // remove this before production
            new MoveElevatorAndALgae(m_aas, m_es, Constants.E_STATE_ALGAE_LOW).withTimeout(2),
            new VisionPureAutoCommand(m_dts, m_ots, -55 / Constants.INCHES_PER_METER, -70 / Constants.INCHES_PER_METER, -135),
            new InstantCommand(()->m_ais.inAlgaeIntake()).withTimeout(0.2),
            new MoveElevatorAndALgae(m_aas, m_es, Constants.E_STATE_ALGAE_HIGH).withTimeout(2),
            new VisionPureAutoCommand(m_dts, m_ots, 5, 0, (-1 *(20 + 3)) - 8, 0),
            new WaitCommand(0.3),
            new InstantCommand(()->m_ais.holdAlgaeIntake()).withTimeout(1),
            new VisionPureAutoCommand(m_dts, m_ots, 0, -15 / Constants.INCHES_PER_METER, 0), // move back 15 inches
            new VisionPureAutoCommand(m_dts, m_ots, 0, 0, 120), // rotate towards barge
            new MoveElevatorAndALgae(m_aas, m_es, new Constants.ElevatorState(16395-1500+9000-((2000/3) * 11), 0, 20)).withTimeout(0.1),
            new VisionPureAutoCommand(m_dts, m_ots, 0, (50+29) / Constants.INCHES_PER_METER, 0), // move towards barge
            new MoveElevatorAndALgae(m_aas, m_es, Constants.E_STATE_ALGAE_BARGE),
            new InstantCommand(()->m_ais.outAlgaeIntake()),
            new WaitCommand(1),
            new InstantCommand(()->m_ais.stopAlgaeIntake())





            //,
            // new InstantCommand(()-> m_dts.resetAngle()).withTimeout(0.1),
            // new InstantCommand(()-> m_dts.zeroOdometry()).withTimeout(0.1),
            // m_dts.createPath(
            //     new Pose2d(0,0,new Rotation2d(Math.toRadians(-90))), 
            //     new Translation2d(0, -0.5), 
            //     new Pose2d(0,-1, new Rotation2d(Math.toRadians(-90))),
            //     180
            // ),
            // new InstantCommand(()->m_dts.stopMotors()).withTimeout(0.1)
            // new WaitCommand(.2),
            // new VisionPureAutoCommand(m_dts, m_ots, 14),
            // new WaitCommand(.2),
            // new VisionPureAutoCommand(m_dts, m_ots, 14, -10, -90, -80),
            // new WaitCommand(.2),
            // new VisionPureAutoCommand(m_dts, m_ots, 8)
        );
    }

    public Command autoGrabAlgaeReef(){
        return new SequentialCommandGroup(
            //new MoveElevatorAndALgae(m_aas, m_es, Constants.E_STATE_ALGAE_LOW).withTimeout(1),
            new InstantCommand(()->m_ais.inAlgaeIntake()).withTimeout(0.2),
            // new ParallelCommandGroup(
            new VisionPureAutoCommand(m_dts, m_ots, 8, 0, (-1 *(20 + 3)) - 8, 0),
            //     new InstantCommand(()->m_ais.inAlgaeIntake()).withTimeout(10)
            // ),
            new WaitCommand(0.3),
            new InstantCommand(()->m_ais.holdAlgaeIntake()).withTimeout(1)

            
            //,
            // new InstantCommand(()-> m_dts.resetAngle()).withTimeout(0.1),
            // new InstantCommand(()-> m_dts.zeroOdometry()).withTimeout(0.1),
            // m_dts.createPath(
            //     new Pose2d(0,0,new Rotation2d(Math.toRadians(-90))), 
            //     new Translation2d(0, -0.5), 
            //     new Pose2d(0,-1, new Rotation2d(Math.toRadians(-90))),
            //     180
            // ),
            // new InstantCommand(()->m_dts.stopMotors()).withTimeout(0.1)
            // new WaitCommand(.2),
            // new VisionPureAutoCommand(m_dts, m_ots, 14),
            // new WaitCommand(.2),
            // new VisionPureAutoCommand(m_dts, m_ots, 14, -10, -90, -80),
            // new WaitCommand(.2),
            // new VisionPureAutoCommand(m_dts, m_ots, 8)
        );
    }
}
