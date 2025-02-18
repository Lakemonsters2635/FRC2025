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
import frc.robot.subsystems.ObjectTrackerSubsystem.Detection;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ObjectTrackerSubsystem;

public class VisionAutoCommand extends Command {
  /** Creates a new VisionAutoCommand. */
  DrivetrainSubsystem m_dts;
  ObjectTrackerSubsystem m_ots;
  double visionX;
  double visionY;
  double visionZ;
  double visionYa;
  Pose2d fieldDeltaPose;
  double fieldX;
  double fieldY;
  int m_tagID;

  double m_xPrime;
  double m_zPrime;
  double m_finalYa;

  double heading; // This really shouldn't be a class variable

  // double xPrime;
  // double zPrime;
  // double finalYa;

  // TODO: make this work again without a tagId
  public VisionAutoCommand(DrivetrainSubsystem dts, ObjectTrackerSubsystem ots) {
    m_dts = dts;
    m_ots = ots;

    // this.xPrime = xPrime0;
    // this.zPrime = zPrime0;
    // this.finalYa = finalYa0;
    addRequirements(m_dts, m_ots);
  }

  public VisionAutoCommand(DrivetrainSubsystem dts, ObjectTrackerSubsystem ots, int tagID) {
    m_dts = dts;
    m_ots = ots;
    m_tagID = tagID;

    m_xPrime = 0;
    m_zPrime = -40;
    m_finalYa = 0;

    // this.xPrime = xPrime0;
    // this.zPrime = zPrime0;
    // this.finalYa = finalYa0;
    addRequirements(m_dts, m_ots);
  }

  public VisionAutoCommand(DrivetrainSubsystem dts, ObjectTrackerSubsystem ots, int tagID, double xPrime, double zPrime, double finalYa) {
    m_dts = dts;
    m_ots = ots;
    m_tagID = tagID;
    m_xPrime = xPrime;
    m_zPrime = zPrime;
    m_finalYa = finalYa;

    addRequirements(m_dts, m_ots);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Don't need to get m_ots.data() because it is already called in Robot.java periodic

    try{
      // visionX = m_ots.getVisionX(m_tagID);
      // visionY = m_ots.getVisionY(m_tagID);
      // visionZ = m_ots.getVisionZ(m_tagID);
      // visionYa = m_ots.getVisionYa(m_tagID);

      SmartDashboard.putNumber("Robot x", m_dts.getPose().getX());
      SmartDashboard.putNumber("Robot y", m_dts.getPose().getY());
      SmartDashboard.putNumber("Robot rot", m_dts.getPose().getRotation().getDegrees());

      // no need to add a small value for xPrime since visionCreatePath takes care of it
      visionCreatePath(m_xPrime, m_zPrime, m_finalYa, m_tagID).schedule();
      System.out.println("Scheduled " + m_tagID + "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    }
    catch(Exception e) {
      System.out.println(e);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // xPrime = 23.5;
    // zPrime = -16.5;
    // finalYa = 0;
    // visionAutoData(23.5, -16.5, 45);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_dts.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
  
  /*
   * This function takes the delta target x, z and yaw values and returns them in terms of field coordinates
   * Note: for the closest april tag use -1 as a parameter to tagId
   */
  public Pose2d visionAutoData(double xPrime, double zPrime, double finalYa, int tagId){
    try{
      Detection detectionObject;
      if(tagId == -1){
        detectionObject = m_ots.getNearestAprilTagDetection();
      }
      else{
        detectionObject = m_ots.getSpecificAprilTag(tagId);
      }

      visionX = detectionObject.x;
      visionZ = detectionObject.z;
      visionY = detectionObject.y;
      visionYa = detectionObject.ya;
    }
    catch(Exception e){
      System.out.println("VisionAutoCommand.visionAutoData(): failed to get vision");
    }

    visionYa*=-1;
    double x_vt = xPrime * Math.cos(Math.toRadians(visionYa)) + -zPrime * Math.sin(Math.toRadians(visionYa));
    double z_vt = xPrime * Math.sin(Math.toRadians(visionYa)) + zPrime * Math.cos(Math.toRadians(visionYa));

    double deltaRobotX = -1 * (visionX + x_vt);
    double deltaRobotY = -1 * (visionZ + z_vt);

    // corrects for the camera position TODO: does this need to be meters for the field
    deltaRobotX += -8.5;
    deltaRobotY += -12.875;

    double botRadians = Units.degreesToRadians(m_dts.getPose().getRotation().getDegrees());
    double angleOffset = -Units.degreesToRadians(90); 

    // double heading = Math.atan(deltaRobotX/deltaRobotY)+botRadians+ angleOffset;
    heading = Math.atan(deltaRobotX/Math.abs(deltaRobotY))+botRadians+ angleOffset;
    double finalAngle = visionYa + finalYa + Units.radiansToDegrees(botRadians);

    // double deltaFieldX = ((deltaRobotX*Math.cos(transformationAngle))+ -(deltaRobotY*Math.sin(transformationAngle)));
    // double deltaFieldY = (deltaRobotX*Math.sin(transformationAngle))+ (deltaRobotY*Math.cos(transformationAngle));

    // We need to change the direction of the botRadions to get the correct transformation
    // TODO: This needs to be documented with drawings and pictures.
    double transformationAngle = botRadians;

    double deltaFieldX = ((deltaRobotX*Math.cos(transformationAngle))+ -(deltaRobotY*Math.sin(transformationAngle)));
    double deltaFieldY = (deltaRobotX*Math.sin(transformationAngle))+ (deltaRobotY*Math.cos(transformationAngle));

    SmartDashboard.putNumber("x_vt", x_vt);
    SmartDashboard.putNumber("z_vt", z_vt);
    SmartDashboard.putNumber("xPrime", xPrime);
    SmartDashboard.putNumber("zPrime", zPrime);
    SmartDashboard.putNumber("deltaRobotX", deltaRobotX);
    SmartDashboard.putNumber("deltaRobotY", deltaRobotY);
    SmartDashboard.putNumber("visionAuto.botRadians", botRadians);
    SmartDashboard.putNumber("visionAuto.heading", heading);
    SmartDashboard.putNumber("deltaFieldX", deltaFieldX);
    SmartDashboard.putNumber("deltaFieldY", deltaFieldY);
    SmartDashboard.putNumber("finalAngle", finalAngle);
    SmartDashboard.putNumber("visionAutoDataYa", visionYa);
    SmartDashboard.putNumber("botRadians degrees", Units.radiansToDegrees(botRadians));

    return new Pose2d(
      Units.inchesToMeters(deltaFieldX), 
      Units.inchesToMeters(deltaFieldY), 
      new Rotation2d(Units.degreesToRadians(finalAngle))
    );
  }

  /*
   * visionAutoData with tagId set to -1 to get the closest april tag
   */
  public Pose2d visionAutoData(double xPrime, double zPrime, double finalYa){
    return visionAutoData(xPrime, zPrime, finalYa, -1);
  }

  /*
   * Uses the visionAutoData function to create a path for the robot to move to a certain position from the closest april tag
   * @param xPrime the x distance from the april tag
   * @param zPrime the z distance from the april tag
   * @param finalYa the final yaw angle for the robot to face
   */
  public Command visionCreatePath(double xPrime, double zPrime, double finalYa){
    // since it is known that xPrime must be non-zero, we should just add 0.000001 to prevent
    // a division by zero.  This protects us from inadvertently providing an invalid answer and allows 
    // us to specify 0 for xPrime when this function is called which is more intuitive than forcing the 
    // user of this function to enter 0.0001 manually to avoid an error.

    //when defining zPrime and xPrime zPrime is positive going behind the april tag and xPrime is positive right of the april tag
    xPrime += 0.00000112358;

    return visionCreatePath(xPrime, zPrime, finalYa, -1);
  }

  /*
   * Uses the visionAutoData function to create a path for the robot the robot to move to a certain position from a specific april tag
   * @param xPrime the x distance from the april tag
   * @param zPrime the z distance from the april tag
   * @param finalYa the final yaw angle for the robot to face
   * @param tagID the id of the april tag
   */
  public Command visionCreatePath(double xPrime, double zPrime, double finalYa, int tagID){
    // since it is known that xPrime must be non-zero, we should just add 0.000001 to prevent
    // a division by zero.  This protects us from inadvertently providing an invalid answer and allows 
    // us to specify 0 for xPrime when this function is called which is more intuitive than forcing the 
    // user of this function to enter 0.0001 manually to avoid an error.

    //when defining zPrime and xPrime zPrime is positive going behind the april tag and xPrime is positive right of the april tag
    xPrime += 0.00000112358;
    
    fieldDeltaPose = visionAutoData(xPrime, zPrime, finalYa, tagID);

    return new SequentialCommandGroup(
      new InstantCommand(()->SmartDashboard.putNumber("dts.getPose() x before",m_dts.getPose().getX())),
      new InstantCommand(()->SmartDashboard.putNumber("dts.getPose() y before",m_dts.getPose().getY())),
      new InstantCommand(()->SmartDashboard.putNumber("dts.getPose() rotation before",m_dts.getPose().getRotation().getDegrees())),
      m_dts.createVisionPath(
        new Pose2d(
          0, //botPose.getX(), 
          0, //botPose.getY(), 
          new Rotation2d(heading)   // TODO need to explain this rotation offset and point to docs
        ), 
        new Translation2d(
          (fieldDeltaPose.getX()/2), 
          (fieldDeltaPose.getY()/2)
        ), 
        new Pose2d(
          fieldDeltaPose.getX(),
          fieldDeltaPose.getY(), 
          new Rotation2d(heading)
        ),
        fieldDeltaPose.getRotation().getDegrees()
        // finalAngle //heading+(Math.PI/2)
        // ,true
      ),
      new InstantCommand(()->m_dts.stopMotors()),
      new InstantCommand(()->SmartDashboard.putNumber("dts.getPose() x after",m_dts.getPose().getX())),
      new InstantCommand(()->SmartDashboard.putNumber("dts.getPose() y after",m_dts.getPose().getY())),
      new InstantCommand(()->SmartDashboard.putNumber("dts.getPose() rotation after",m_dts.getPose().getRotation().getDegrees()))
    );
  }
}
