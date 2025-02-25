// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

/** Add your docs here. */
public class Autos {
    DrivetrainSubsystem m_dts;
    public Autos(DrivetrainSubsystem dts) {
        m_dts = dts;
    }

    public Command goStraight(){
        return m_dts.createPath(
            new Pose2d(0,0, new Rotation2d(Units.degreesToRadians(90))), 
            new Translation2d(0, 0.5), 
            new Pose2d(0, 1, new Rotation2d(Units.degreesToRadians(90)))
        );
    }
}
