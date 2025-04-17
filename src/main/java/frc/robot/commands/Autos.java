// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
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

    public Autos(DrivetrainSubsystem dts, ObjectTrackerSubsystem ots, ElevatorSubsystem es, AlgaeArmSubsystem aas,
            AlgaeIntakeSubsystem ais) {
        m_dts = dts;
        m_ots = ots;
        m_es = es;
        m_aas = aas;
        m_ais = ais;
    }

    public int redBlueApriltags(int id) {
        // https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/Apriltag_Images_and_User_Guide.pdf
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            return id;
        } else {
            switch (id) {
                case 21:
                    return 10;
                case 20:
                    return 11;
                case 19:
                    return 6;
                case 14:
                    return 5; // Barge
                case 4:
                    return 15; // Barge - other side
                default:
                    return -1;
            }
        }
    }

    public Command goStraight() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> m_dts.stopMotors()),
                new InstantCommand(() -> m_dts.setFollowJoystick(false)),
                new InstantCommand(() -> m_es.setElevatorTarget(Constants.E_STATE_AUTO_FINAL.ELEVATOR_POSITION)),
                new InstantCommand(() -> m_aas.setArmPosition(Constants.E_STATE_AUTO_FINAL.ALGAE_ARM_ANGLE)),
                m_dts.createPath(
                        new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(-90))),
                        new Translation2d(0, 0.5),
                        new Pose2d(0, 1, new Rotation2d(Units.degreesToRadians(-90)))),
                new InstantCommand(() -> m_dts.setFollowJoystick(true)),

                // m_dts.createPath(
                // new Pose2d(0,2.8, new Rotation2d(Units.degreesToRadians(90))),
                // new Translation2d(0, 2.9),
                // new Pose2d(0, 3, new Rotation2d(Units.degreesToRadians(90)))
                // ),
                new InstantCommand(() -> m_dts.stopMotors()),
                new InstantCommand(() -> m_dts.resetAngle(180)),
                new InstantCommand(() -> m_dts.zeroOdometry()));
    }

    public Command visionAuto() {
        return new SequentialCommandGroup(
                new VisionAutoCommand(m_dts, m_ots));
    }

    // Source line-up auto
    public Command sourceLineup() {
        return new SequentialCommandGroup(
                // new VisionAutoCommand(m_dts, m_ots, Constants.SOURCE_TAG_IDS, 0.0, 0.0, 0.0)
                // // TODO: determine these values
                new VisionAutoCommand(m_dts, m_ots, -1, 0.0, -1 * (5 + 30), 0.0) // TODO: determine these values
        // new VisionAutoCommand(m_dts, m_ots, Constants.SOURCE_TAG_IDS, 0.0, -1*(5+30),
        // 0.0) // TODO: determine these values
        );
    }

    // Corral line-up left
    public Command reefCorralLeft() {
        return new SequentialCommandGroup(
                new VisionAutoCommand(m_dts, m_ots, -1, 13, -1 * (20 + 11), 0.0));
    }

    // Corral line-up right
    public Command reefCorralRight() {
        return new SequentialCommandGroup(
                new VisionAutoCommand(m_dts, m_ots, -1, -13, -1 * (20 + 11), 0) // TODO
        );
    }

    // Corral line-up algae
    public Command reefAlgae() {
        return new SequentialCommandGroup(
                // new InstantCommand(()-> m_dts.stopMotors()),
                // new WaitCommand(0.5),
                // new InstantCommand(()-> SmartDashboard.putString("reefAlgae", "runs")),
                // new VisionAutoCommand(m_dts, m_ots, 8)
                new VisionAutoCommand(m_dts, m_ots, -1, 0, -1 * (20 + 3), 0));
    }
    // public Command reefAlgae(){
    // return new SequentialCommandGroup(
    // // new InstantCommand(()-> m_dts.stopMotors()),
    // // new WaitCommand(0.5),
    // // new InstantCommand(()-> SmartDashboard.putString("reefAlgae", "runs")),
    // // new VisionAutoCommand(m_dts, m_ots, 8)
    // new VisionAutoCommand(m_dts, m_ots, -1, 0, -1 *(20 + 12), 0)
    // );
    // }
    // public Command reefAlgae(){
    // return new SequentialCommandGroup(
    // new VisionAutoCommand(m_dts, m_ots, Constants.REEF_TAG_IDS, 0, 48, 0)
    // );
    // }

    // Line-up processor
    public Command processorAlgae() {
        return new SequentialCommandGroup(
                new VisionAutoCommand(m_dts, m_ots, -1, 0, 0, 0));
    }

    public Command closestAprilTag() {
        return new SequentialCommandGroup(
                new VisionAutoCommand(m_dts, m_ots, -1, 0, -30, 0));
    }

    public Command bargeAlgae() {
        return new SequentialCommandGroup(
                new VisionPureAutoCommand(m_dts, m_ots, 3),
                new WaitCommand(0.2)
        // TODO: work on this
        );
    }

    public Command multipleVisionAutos() {
        return new SequentialCommandGroup(
                new VisionPureAutoCommand(m_dts, m_ots, 3),
                new WaitCommand(2),
                new InstantCommand(() -> m_dts.resetOdometry(new Pose2d(0, 0, m_dts.getPose().getRotation()))),
                // We need to reset odometry before we do a createPath but do not want to reset
                // the angle
                m_dts.createPath(
                        new Pose2d(0, 0, new Rotation2d(-90)),
                        new Translation2d(0, -0.75),
                        new Pose2d(0, -1.5, new Rotation2d(-90))),
                new WaitCommand(2),
                new VisionPureAutoCommand(m_dts, m_ots, 8));
    }

    public Command chainPureMultiVision() {
        return new SequentialCommandGroup(
                new VisionPureAutoCommand(m_dts, m_ots, 8),
                new WaitCommand(.2),
                new VisionPureAutoCommand(m_dts, m_ots, 8, 0, -90, 70),
                new WaitCommand(.2),
                new VisionPureAutoCommand(m_dts, m_ots, 14),
                new WaitCommand(.2),
                new VisionPureAutoCommand(m_dts, m_ots, 14, -10, -90, -80),
                new WaitCommand(.2),
                new VisionPureAutoCommand(m_dts, m_ots, 8));
    }

    public Command centerReef() {
        return new SequentialCommandGroup(
                // First algae pickup
                new MoveElevatorAndALgae(m_aas, m_es, Constants.E_STATE_ALGAE_LOW).withTimeout(0.021),
                new InstantCommand(() -> m_ais.inAlgaeIntake()).withTimeout(0.021),
                new VisionPureAutoCommand(m_dts, m_ots, redBlueApriltags(21), 0, (-1 * (20 + 3)) - 8, 0),
                new WaitCommand(0.1), // might not need it
                new InstantCommand(() -> m_ais.holdAlgaeIntake()).withTimeout(0.021),
                // First barge score
                new VisionPureAutoCommand(m_dts, m_ots, 0, -26 / Constants.INCHES_PER_METER, 0),
                // Note: the following line is esentially a paralel command since the subsystem
                // takes care of moving elevator and arm
                new MoveElevatorAndALgae(m_aas, m_es,
                        new Constants.ElevatorState(16395 - 1500 + 9000 - ((2000 / 3) * 11), 0, 20)).withTimeout(0.021),
                new VisionPureAutoCommand(m_dts, m_ots, 40 / Constants.INCHES_PER_METER,
                        (-24 - 8) / Constants.INCHES_PER_METER, -160),
                new MoveElevatorAndALgae(m_aas, m_es, Constants.E_STATE_ALGAE_BARGE),
                new WaitCommand(0.05),
                new InstantCommand(() -> m_ais.outAlgaeIntake()).withTimeout(0.021),
                new WaitCommand(0.5),
                new InstantCommand(() -> m_ais.stopAlgaeIntake()).withTimeout(0.021),
                new MoveElevatorAndALgae(m_aas, m_es, Constants.E_STATE_DRIVE).withTimeout(0.021),
                new VisionPureAutoCommand(m_dts, m_ots, 10 / Constants.INCHES_PER_METER,
                        (-25) / Constants.INCHES_PER_METER, 0).withTimeout(2));
    }

    // public Command autoReefAndBarge(){
    // return new SequentialCommandGroup(
    // // First algae pickup
    // new MoveElevatorAndALgae(m_aas, m_es,
    // Constants.E_STATE_ALGAE_LOW).withTimeout(0.021),
    // new InstantCommand(()->m_ais.inAlgaeIntake()).withTimeout(0.021),
    // new VisionPureAutoCommand(m_dts, m_ots, 8, 0, (-1 *(20 + 3)) - 8, 0),
    // new WaitCommand(0.1), // might not need it
    // new InstantCommand(()->m_ais.holdAlgaeIntake()).withTimeout(0.021),
    // // First barge score
    // new VisionPureAutoCommand(m_dts, m_ots, 0, -26 / Constants.INCHES_PER_METER,
    // 0),
    // // Note: the following line is esentially a paralel command since the
    // subsystem takes care of moving elevator and arm
    // new MoveElevatorAndALgae(m_aas, m_es, new
    // Constants.ElevatorState(16395-1500+9000-((2000/3) * 11), 0,
    // 20)).withTimeout(0.021),
    // new VisionPureAutoCommand(m_dts, m_ots, 40 / Constants.INCHES_PER_METER,
    // (-32+15) / Constants.INCHES_PER_METER, -160),
    // new MoveElevatorAndALgae(m_aas, m_es, Constants.E_STATE_ALGAE_BARGE),
    // new WaitCommand(0.05),
    // new InstantCommand(()->m_ais.outAlgaeIntake(11)).withTimeout(0.021),
    // new WaitCommand(0.5),
    // new InstantCommand(()->m_ais.stopAlgaeIntake()).withTimeout(0.021),
    // // Second Algae pickup
    // new MoveElevatorAndALgae(m_aas, m_es,
    // Constants.E_STATE_ALGAE_LOW).withTimeout(0.021),
    // new VisionPureAutoCommand(m_dts, m_ots, -52 / Constants.INCHES_PER_METER,
    // (-65+15) / Constants.INCHES_PER_METER, -135),
    // new InstantCommand(()->m_ais.inAlgaeIntake()).withTimeout(0.02),
    // new WaitCommand(0.3+0.2),
    // new MoveElevatorAndALgae(m_aas, m_es,
    // Constants.E_STATE_ALGAE_HIGH).withTimeout(0.021),
    // new VisionPureAutoCommand(m_dts, m_ots, 5, 0, (-1 *(20 + 3)) - 8, 0),
    // new WaitCommand(0.1),
    // new InstantCommand(()->m_ais.holdAlgaeIntake()).withTimeout(0.021),
    // // Second barge score
    // new VisionPureAutoCommand(m_dts, m_ots, 0, -15 / Constants.INCHES_PER_METER,
    // 0), // move back 15 inches
    // new VisionPureAutoCommand(m_dts, m_ots, 0, 0, 120), // rotate towards barge
    // new MoveElevatorAndALgae(m_aas, m_es, new
    // Constants.ElevatorState(16395-1500+9000-((2000/3) * 11), 0,
    // 20)).withTimeout(0.021),
    // new VisionPureAutoCommand(m_dts, m_ots, 0, (59) / Constants.INCHES_PER_METER,
    // 0), // move towards barge
    // new MoveElevatorAndALgae(m_aas, m_es, Constants.E_STATE_ALGAE_BARGE),
    // new InstantCommand(()->m_ais.outAlgaeIntake(7.5)).withTimeout(0.021),
    // new WaitCommand(0.5),
    // new InstantCommand(()->m_ais.stopAlgaeIntake()).withTimeout(0.021),
    // new VisionPureAutoCommand(m_dts, m_ots, 0, -15 / Constants.INCHES_PER_METER,
    // 0),
    // new MoveElevatorAndALgae(m_aas, m_es, Constants.E_STATE_DRIVE)
    // );
    // }
    public Command autoReefAndBarge() {
        return new SequentialCommandGroup(
                // First algae pickup
                new MoveElevatorAndALgae(m_aas, m_es, Constants.E_STATE_ALGAE_LOW).withTimeout(0.021),
                new InstantCommand(() -> m_ais.inAlgaeIntake()).withTimeout(0.021),
                new VisionPureAutoCommand(m_dts, m_ots, redBlueApriltags(21), 0, (-1 * (20 + 3)) - 8, 0),
                new WaitCommand(0.1), // might not need it
                new InstantCommand(() -> m_ais.holdAlgaeIntake()).withTimeout(0.021),
                // First barge score
                new VisionPureAutoCommand(m_dts, m_ots, 0, -26 / Constants.INCHES_PER_METER, 0),
                // Note: the following line is esentially a paralel command since the subsystem
                // takes care of moving elevator and arm
                new MoveElevatorAndALgae(m_aas, m_es,
                        new Constants.ElevatorState(16395 - 1500 + 9000 - ((2000 / 3) * 11), 0, 20)).withTimeout(0.021),
                new VisionPureAutoCommand(m_dts, m_ots, 40 / Constants.INCHES_PER_METER,
                        (-24 - 8) / Constants.INCHES_PER_METER, -160),
                new MoveElevatorAndALgae(m_aas, m_es, Constants.E_STATE_ALGAE_BARGE),
                new WaitCommand(0.05),
                new InstantCommand(() -> m_ais.outAlgaeIntake()).withTimeout(0.021),
                new WaitCommand(0.5),
                new InstantCommand(() -> m_ais.stopAlgaeIntake()).withTimeout(0.021),
                // Second Algae pickup
                new MoveElevatorAndALgae(m_aas, m_es, Constants.E_STATE_ALGAE_LOW).withTimeout(0.021),
                new VisionPureAutoCommand(m_dts, m_ots, -52 / Constants.INCHES_PER_METER,
                        -65 / Constants.INCHES_PER_METER, -135),
                new InstantCommand(() -> m_ais.inAlgaeIntake()).withTimeout(0.02),
                new WaitCommand(0.3 + 0.2),
                new MoveElevatorAndALgae(m_aas, m_es, Constants.E_STATE_ALGAE_HIGH).withTimeout(0.021),
                new VisionPureAutoCommand(m_dts, m_ots, redBlueApriltags(20), 0, (-1 * (20 + 3)) - 8, 0),
                new WaitCommand(0.1),
                new InstantCommand(() -> m_ais.holdAlgaeIntake()).withTimeout(0.021),
                // Second barge score
                new VisionPureAutoCommand(m_dts, m_ots, 0, -15 / Constants.INCHES_PER_METER, 0), // move back 15 inches
                new VisionPureAutoCommand(m_dts, m_ots, 0, 0, 120), // rotate towards barge
                new MoveElevatorAndALgae(m_aas, m_es,
                        new Constants.ElevatorState(16395 - 1500 + 9000 - ((2000 / 3) * 11), 0, 20)).withTimeout(0.021),
                new VisionPureAutoCommand(m_dts, m_ots, 0, (50 + 29 - 5 - 15) / Constants.INCHES_PER_METER, 0), // move
                                                                                                                // towards
                                                                                                                // barge
                new MoveElevatorAndALgae(m_aas, m_es, Constants.E_STATE_ALGAE_BARGE),
                new InstantCommand(() -> m_ais.outAlgaeIntake(7.5)).withTimeout(0.021),
                new WaitCommand(1),
                new InstantCommand(() -> m_ais.stopAlgaeIntake()).withTimeout(0.021),
                new MoveElevatorAndALgae(m_aas, m_es, Constants.E_STATE_DRIVE));
    }

    public Command autoReefAndBargeRight() {
        return new SequentialCommandGroup(
                // First algae intake
                new InstantCommand(() -> m_ais.inAlgaeIntake()).withTimeout(0.02),
                new MoveElevatorAndALgae(m_aas, m_es, Constants.E_STATE_ALGAE_HIGH).withTimeout(0.8), // can decrease
                                                                                                      // the timeout
                                                                                                      // (risky)
                new VisionPureAutoCommand(m_dts, m_ots, redBlueApriltags(20), 0, (-1 * (20 + 3)) - 8, 0),
                new WaitCommand(0.1),
                new InstantCommand(() -> m_ais.holdAlgaeIntake()).withTimeout(0.021),
                // First barge score
                new VisionPureAutoCommand(m_dts, m_ots, 0, -15 / Constants.INCHES_PER_METER, 0), // move back 15 inches
                // new VisionPureAutoCommand(m_dts, m_ots, 0, 0, 120), // rotate towards barge
                new MoveElevatorAndALgae(m_aas, m_es,
                        new Constants.ElevatorState(16395 - 1500 + 9000 - ((2000 / 3) * 11), 0, 20)).withTimeout(0.021),
                new VisionPureAutoCommand(m_dts, m_ots,
                        (50 + 29 - 5 - 15) / Constants.INCHES_PER_METER * Math.cos(Math.toRadians(120)),
                        -1 * (50 + 29 - 5 - 15) / Constants.INCHES_PER_METER * Math.sin(Math.toRadians(120)), 120), // move
                                                                                                                    // towards
                                                                                                                    // barge
                new MoveElevatorAndALgae(m_aas, m_es, Constants.E_STATE_ALGAE_BARGE),
                new InstantCommand(() -> m_ais.outAlgaeIntake(7.5)).withTimeout(0.021),
                new WaitCommand(0.35), // Could be less, but if it breaks the whole auto would be broken
                new InstantCommand(() -> m_ais.stopAlgaeIntake()).withTimeout(0.021),
                // Second algae intake
                new MoveElevatorAndALgae(m_aas, m_es, Constants.E_STATE_ALGAE_LOW).withTimeout(0.4),
                new VisionPureAutoCommand(m_dts, m_ots, -5 / Constants.INCHES_PER_METER,
                        -130 / Constants.INCHES_PER_METER, -75.0),
                new InstantCommand(() -> m_ais.inAlgaeIntake()).withTimeout(0.02),
                new WaitCommand(0.5), // Doesn't need this timeout
                new VisionPureAutoCommand(m_dts, m_ots, redBlueApriltags(19), -5, (-1 * (20 + 3)) - 8 + 2, 0), // little
                                                                                                               // bit
                                                                                                               // further
                                                                                                               // than
                                                                                                               // the
                                                                                                               // first
                                                                                                               // algae
                new WaitCommand(0.2),
                new InstantCommand(() -> m_ais.holdAlgaeIntake()).withTimeout(0.1),
                // Second barge Score
                new VisionPureAutoCommand(m_dts, m_ots, 0, -20 / Constants.INCHES_PER_METER, 0),
                new VisionPureAutoCommand(m_dts, m_ots, 0, 0, 60),
                new WaitCommand(0.2),
                new VisionPureAutoCommand(m_dts, m_ots, 0, 110 / Constants.INCHES_PER_METER, 0),
                new WaitCommand(0.2),
                new MoveElevatorAndALgae(m_aas, m_es, Constants.E_STATE_ALGAE_BARGE).withTimeout(1) // Doesn't need this
                                                                                                    // timeout
        // new InstantCommand(()->m_ais.outAlgaeIntake(10)),
        // new WaitCommand(0.35), // Could be less but doesn't matter for the time
        // new InstantCommand(()->m_ais.stopAlgaeIntake()),
        // new MoveElevatorAndALgae(m_aas, m_es, Constants.E_STATE_DRIVE)
        );
    }

    public Command autoGrabAlgaeReefLow() {
        return new SequentialCommandGroup(
                new MoveElevatorAndALgae(m_aas, m_es, Constants.E_STATE_ALGAE_LOW_TELE),
                new InstantCommand(() -> m_ais.inAlgaeIntake()).withTimeout(0.2),
                new VisionPureAutoCommand(m_dts, m_ots, -1, 0, (-1 * (20 + 3)) - 8, 0),
                new WaitCommand(0.3),
                new InstantCommand(() -> m_ais.holdAlgaeIntake()).withTimeout(0.1));
    }

    public Command autoGrabAlgaeReefHigh() {
        return new SequentialCommandGroup(
                new MoveElevatorAndALgae(m_aas, m_es, Constants.E_STATE_ALGAE_HIGH_TELE),
                new InstantCommand(() -> m_ais.inAlgaeIntake()).withTimeout(0.2),
                new VisionPureAutoCommand(m_dts, m_ots, -1, 0, (-1 * (20 + 3)) - 8, 0),
                new WaitCommand(0.3),
                new InstantCommand(() -> m_ais.holdAlgaeIntake()).withTimeout(0.1));
    }

    public Command autoGrabAlgaeGround() {
        return new SequentialCommandGroup(
                new MoveElevatorAndALgae(m_aas, m_es, Constants.E_STATE_ALGAE_PICKUP_GROUND).withTimeout(0.1),
                new InstantCommand(() -> m_ais.inAlgaeIntake()).withTimeout(0.2),
                new VisionPureAutoCommand(m_dts, m_ots, "algae", 0, (-1 * (20 + 13)), 0), // the YOLO_OBJECT could be
                                                                                          // anything other than ""
                new WaitCommand(0.3),
                new InstantCommand(() -> m_ais.holdAlgaeIntake()).withTimeout(0.1),
                new InstantCommand(() -> m_dts.stopMotors()).withTimeout(0.1)
        // commented out so the arms do not raise up if the algae was missed
        // , new MoveElevatorAndALgae(m_aas, m_es, new
        // Constants.ElevatorState(Constants.E_STATE_ALGAE_PICKUP_GROUND.ELEVATOR_POSITION,
        // 0.0, 20.0)).withTimeout(0.1)
        );
    }
}
