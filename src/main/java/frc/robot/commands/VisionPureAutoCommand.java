// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.function.ObjDoubleConsumer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
// import frc.robot.subsystems.ObjectTrackerSubsystem.Detection;
import frc.robot.subsystems.Detection;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ObjectTrackerSubsystem;

public class VisionPureAutoCommand extends Command {
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
  int m_tagID = 8; // When initialized at -2 so that if the user wants to input tagIDs, it is going to chose multiple instead of a sigular tagID
  int[] m_tagIDs;

  double m_xPrime;
  double m_zPrime;
  double m_finalYa;

  double m_x_target;
  double m_y_target;
  double m_rot_target;
  double m_c_target; //hypotneuse

  double m_x_start;
  double m_y_start;
  double m_rot_start;

  double heading; // This really shouldn't be a class variable

  // double xPrime;
  // double zPrime;
  // double finalYa;

  // These PID values for x and y convert an error in meters into a commanded speed.
  // if kp == 2, then a 1 meter error in position will command a 2 m/s speed to close the error
  PIDController m_visionSwerveController_x = new PIDController(10, 0, 0); //5 2 0
  PIDController m_visionSwerveController_y = new PIDController(10, 0, 0);

  // 
  PIDController m_visionSwerveController_rot = new PIDController(4,0, 0);

  double PURE_VISION_MAX_M_PER_SEC = 2; //1.5;
  double PURE_VISION_MAX_RAD_PER_SEC = Math.PI; // normal limit is Math.PI radians per second

  double m_fb_x = 0.;
  double m_fb_y = 0.;
  double m_fb_rot = 0.;
  boolean isVisionAuto = false;

  public VisionPureAutoCommand(DrivetrainSubsystem dts, ObjectTrackerSubsystem ots, int tagID) {
    m_dts = dts;
    m_ots = ots;
    m_tagID = tagID;

    m_xPrime = 0;
    m_zPrime = -46;
    m_finalYa = 0;
    isVisionAuto = true;

    // this.xPrime = xPrime0;
    // this.zPrime = zPrime0;
    // this.finalYa = finalYa0;
    addRequirements(m_dts, m_ots);

    // m_visionSwerveController_x.setTolerance(0.1); // in meters
    // m_visionSwerveController_y.setTolerance(0.01);
    // m_visionSwerveController_rot.setTolerance(0.01);
    
  }

  public VisionPureAutoCommand(DrivetrainSubsystem dts, ObjectTrackerSubsystem ots, int tagID, double xPrime, double zPrime, double finalYa) {
    m_dts = dts;
    m_ots = ots;
    m_tagID = tagID;
    m_xPrime = xPrime;
    m_zPrime = zPrime;
    m_finalYa = finalYa;
    isVisionAuto = true;

    addRequirements(m_dts, m_ots);
  }
  //This constructor manually defines target distances without the use of vision
  public VisionPureAutoCommand(DrivetrainSubsystem dts, ObjectTrackerSubsystem ots, double xTarget, double yTarget, double rotTarget){
    m_dts = dts;
    m_ots = ots;
    m_x_target = xTarget;
    m_y_target = yTarget;
    m_rot_target = rotTarget;
    isVisionAuto = false;   
    
    addRequirements(m_dts, m_ots);
  }



//   public VisionPureAutoCommand(DrivetrainSubsystem dts, ObjectTrackerSubsystem ots, int[] tagIDs, double xPrime, double zPrime, double finalYa) {
//     m_dts = dts;
//     m_ots = ots;
//     m_tagIDs = tagIDs;
//     m_xPrime = xPrime;
//     m_zPrime = zPrime;
//     m_finalYa = finalYa;

//     addRequirements(m_dts, m_ots);
//   }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // TODO: need to set the field x, y, rotation target.
    // TODO the x,y,rot will be based on the initial state + what was defined in the constructor based on desired relative positoin from the april tag.
    m_dts.stashAngle();
    m_dts.resetAngle();    
    m_dts.zeroOdometry();
    if(isVisionAuto){
      Pose2d fieldDeltaPose = visionAutoData(m_xPrime, m_zPrime, m_finalYa, m_tagID);
      m_x_target = fieldDeltaPose.getX();
      m_y_target = fieldDeltaPose.getY();
      m_rot_target = fieldDeltaPose.getRotation().getDegrees();
    }
    m_c_target = Math.sqrt(Math.pow(m_x_target, 2) + Math.pow(m_y_target, 2));


    SmartDashboard.putNumber("xPidTarget", m_x_target);
    SmartDashboard.putNumber("yPidTarget", m_y_target);
    SmartDashboard.putNumber("rotPidTarget", m_rot_target);
    SmartDashboard.putNumber("cPidTarget", m_c_target);


    // get the starting pose so we can calculate fade-in for speed
    m_x_start = m_dts.getPose().getX();
    m_y_start = m_dts.getPose().getY();
    m_rot_start = m_dts.getPose().getRotation().getDegrees();

    // NOTE: actual pose of the robot will be updated in execute.


    // Don't need to get m_ots.data() because it is already called in Robot.java periodic

    m_dts.setFollowJoystick(false);

    try{
      // visionX = m_ots.getVisionX(m_tagID);
      // visionY = m_ots.getVisionY(m_tagID);
      // visionZ = m_ots.getVisionZ(m_tagID);
      // visionYa = m_ots.getVisionYa(m_tagID);

      SmartDashboard.putNumber("Robot x", m_dts.getPose().getX());
      SmartDashboard.putNumber("Robot y", m_dts.getPose().getY());
      SmartDashboard.putNumber("Robot rot", m_dts.getPose().getRotation().getDegrees());

    //   // no need to add a small value for xPrime since visionCreatePath takes care of it
    //   if (m_tagID == -2) {
    //     visionCreatePath(m_xPrime, m_zPrime, m_finalYa, m_tagID).schedule();
    //   }
    //   else if(m_tagID >= -1){
    //     visionCreatePath(m_xPrime, m_zPrime, m_finalYa, m_tagID).schedule();
    //   }
    //   else{
    //     visionCreatePath(m_xPrime, m_zPrime, m_finalYa, Integer.valueOf(m_ots.getNearestAprilTagDetection(m_tagIDs).objectLabel.substring(10)));
    //   }
    //   System.out.println("Scheduled " + m_tagID + "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    }
    catch(Exception e) {
      System.out.println(e);
    }
    m_dts.stopMotors();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get the vision

    // TODO this pose will be current pose from odemetry.
    double x_pose = m_dts.getPose().getX();
    double y_pose = m_dts.getPose().getY();
    double rot_pose = m_dts.getPose().getRotation().getDegrees();

    // TODO need something to fade-in the acceleration so we don't brown out the  robot.
    // we can do this by remembering how far we are from the start... maybe it is better to take a hack 
    // use m_{xy,yrot}_start for fade-in... need to figure out how far of a distance this needs to be faded in from.
    // this fade in distance likely needs to be an initialization parameter.
    double distanceFromStart = Math.sqrt(Math.pow((x_pose - m_x_start), 2) + Math.pow((y_pose - m_y_start), 2)); //use pythagoream 
    double fadeInDistance = 0.5; //in meters
    double speed_clamp = PURE_VISION_MAX_M_PER_SEC * (distanceFromStart/fadeInDistance);
    // speed_clamp is the max speed that we will go and defaults to 0.5 meters per second, and goes up to 
    // PURE_VISION_MAX_M_PER_SEC once we have gone fadeInDistance meters away from the origin
    speed_clamp = MathUtil.clamp(speed_clamp,0.5, PURE_VISION_MAX_M_PER_SEC); // low: 0.5, high:1.5
    // double x_clamp = speed_clamp * (Math.abs(m_x_target)/m_c_target); //speed_clamp * cos(theta)
    // double y_clamp = speed_clamp * (Math.abs(m_y_target)/m_c_target); //speed_clamp * sin(theta)

    double pid_x_calculate = m_visionSwerveController_x.calculate(x_pose, m_x_target);
    double pid_y_calculate = m_visionSwerveController_y.calculate(y_pose, m_y_target);
    double pid_rot_calculate = m_visionSwerveController_rot.calculate(Math.toRadians(rot_pose), Math.toRadians(m_rot_target));

    double pid_c = Math.sqrt(pid_x_calculate * pid_x_calculate + pid_y_calculate * pid_y_calculate);

    // double x_clamp = speed_clamp * (Math.abs(pid_x_calculate)/pid_c); //speed_clamp * cos(theta)
    // double y_clamp = speed_clamp * (Math.abs(pid_y_calculate)/pid_c); //speed_clamp * sin(theta)

    double x_clamp = (pid_c>speed_clamp) ? 
                        (speed_clamp * (Math.abs(pid_x_calculate)/pid_c)):
                        (speed_clamp); //speed_clamp * cos(theta)

    double y_clamp = (pid_c>speed_clamp) ? 
                        (speed_clamp * (Math.abs(pid_y_calculate)/pid_c)):
                        (speed_clamp); //speed_clamp * cos(theta)

    {
      SmartDashboard.putNumber("pid pid_x_calculate", pid_x_calculate);
      SmartDashboard.putNumber("pid pid_y_calculate", pid_y_calculate);
      SmartDashboard.putNumber("pid distanceFromStart", distanceFromStart);
      SmartDashboard.putNumber("pid speed_clamp", speed_clamp);

      SmartDashboard.putNumber("pid x_clamp", x_clamp);
      SmartDashboard.putNumber("pid y_clamp", y_clamp);
      SmartDashboard.putNumber("pid pid_c", pid_c);

      SmartDashboard.putNumber("pid x_pose", x_pose);
      SmartDashboard.putNumber("pid y_pose", y_pose);
      SmartDashboard.putNumber("pid rot_pose", rot_pose);

      SmartDashboard.putNumber("pid x_target", m_x_target);
      SmartDashboard.putNumber("pid y_target", m_y_target);
      SmartDashboard.putNumber("pid rot_target", m_rot_target);
    }
    

    m_fb_x = MathUtil.clamp(
        pid_x_calculate,
         -1 * x_clamp, x_clamp
    );
    m_fb_y = MathUtil.clamp(
      pid_y_calculate, 
        -1 * y_clamp, y_clamp
    );
    m_fb_rot = MathUtil.clamp(
      pid_rot_calculate, 
        -1 * PURE_VISION_MAX_RAD_PER_SEC, PURE_VISION_MAX_RAD_PER_SEC
    );

    SmartDashboard.putNumber("m_fb_rot", m_fb_rot);
    SmartDashboard.putNumber("m_fb_x", m_fb_x);
    SmartDashboard.putNumber("m_fb_y", m_fb_y);

    // CLAMP the values so they are not too fast
    // TODO look at it to see if we want field relative or robot centric.
    // .    our equeitons simplify if we do robot centric.
    double driveX_Fraction   = m_fb_x   ; // / Constants.maxModuleLinearSpeed;
    double driveY_Fraction   = m_fb_y   ; // / Constants.maxModuleLinearSpeed;
    double driveRot_Fraction = m_fb_rot;

    SmartDashboard.putNumber("driveXSpeedPidAuto", driveX_Fraction);
    SmartDashboard.putNumber("driveYSpeedPidAuto", driveY_Fraction);
    SmartDashboard.putNumber("driveRotSpeedPidAuto", driveRot_Fraction);

    // TODO: fix comments m_dts.drive, we are actually giving the speed not a fraction
    m_dts.drive(driveX_Fraction, driveY_Fraction, driveRot_Fraction, true);
    // xPrime = 23.5;
    // zPrime = -16.5;
    // finalYa = 0;
    // visionAutoData(23.5, -16.5, 45);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_dts.restoreAngle();
    m_dts.stopMotors();
    m_dts.setFollowJoystick(true);
    m_dts.setStopVisionAutoCommand(false);
}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double x_pose = m_dts.getPose().getX();
    double y_pose = m_dts.getPose().getY();
    double rot_pose = m_dts.getPose().getRotation().getDegrees();

    SmartDashboard.putNumber("pid isFinished X", (m_x_target - x_pose));
    SmartDashboard.putNumber("pid isFinished Y", (m_y_target - y_pose));
    SmartDashboard.putNumber("pid isFinished Rot", (m_rot_target - rot_pose));
    // SmartDashboard.putNumber("pid isFinished X", Math.abs(m_x_target - x_pose));
    // SmartDashboard.putNumber("pid isFinished Y", Math.abs(m_y_target - y_pose));
    // SmartDashboard.putNumber("pid isFinished Rot", Math.abs(m_rot_target - rot_pose));

    if (Math.abs(m_x_target - x_pose) < 0.01 && Math.abs(m_y_target - y_pose) < 0.01 && (Math.abs(m_rot_target - rot_pose) % 360) < 1  && Math.abs(m_dts.getYawGyroValue()) < 10) {
      return true;
    }

    // if((m_visionSwerveController_x.atSetpoint() && m_visionSwerveController_y.atSetpoint() && m_visionSwerveController_rot.atSetpoint()) || m_dts.getStopVisionAutoCommand()){
    //   return true;
    // }

    return false;
  }
  
  /*
   * This function takes the delta target x, z and yaw values and returns them in terms of field coordinates
   * Note: for the closest april tag use -1 as a parameter to tagId
   */
  public Pose2d visionAutoData(double xPrime, double zPrime, double finalYa, int tagId){
    boolean notDone = true;
    int i = 0;
    while(notDone)
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
          notDone = false;
        }
        catch(Exception e){
          i++;
          if(i > 100){
            notDone = false;
          }
          System.out.println("VisionAutoCommand.visionAutoData(): failed to get vision" + " i: " + i);
        }

    visionYa*=-1;
    double x_vt = xPrime * Math.cos(Math.toRadians(visionYa)) + -zPrime * Math.sin(Math.toRadians(visionYa));
    double z_vt = xPrime * Math.sin(Math.toRadians(visionYa)) + zPrime * Math.cos(Math.toRadians(visionYa));

    double deltaRobotX = -1 * (visionX + x_vt);
    double deltaRobotY = -1 * (visionZ + z_vt);

    // corrects for the camera position TODO: does this need to be meters for the field
    deltaRobotX += -5;//-8.5;
    deltaRobotY += -14;//-12.875;

    deltaRobotX *=-1;
    deltaRobotY *=-1;

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

    // deltaFieldX *= -1; // When the camera is on the front of the robot
    // deltaFieldY *= -1;

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
      Units.inchesToMeters(deltaRobotX), 
      Units.inchesToMeters(deltaRobotY), 
      // Units.inchesToMeters(deltaFieldX), 
      // Units.inchesToMeters(deltaFieldY), 
      new Rotation2d(Units.degreesToRadians(finalAngle))
    );
  }

  /*
   * visionAutoData with tagId set to -1 to get the closest april tag
   */
  public Pose2d visionAutoData(double xPrime, double zPrime, double finalYa){
    return visionAutoData(xPrime, zPrime, finalYa, -1);
  }
}
