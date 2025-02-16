// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.servohub.config.ServoChannelConfig.PulseRange;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  // joystick channels
  public static final int RIGHT_JOYSTICK_CHANNEL = 1;
  public static final int LEFT_JOYSTICK_CHANNEL = 0;

  public static final double INCHES_PER_METER = 39.37;

  // If you want to change the definition of what is front left etc. 

  // For 180 switch the constants of encoder offsets, motor and encoder ids for front left and back right, and front right and back left
  // Add 180 to all the encoder offsets

  // For rotating front of the robot clockwise 90 degrees for motor and encoder constants and encoder offsets
  // make the front left to front right, front right to back right, back right to back left, back left to front left
  // Subtract 90 to all encoder offsets
  // Switch robot's lenght to width

  // For rotating front of the robot counter-clockwise 90 degrees for motor and encoder constants and encoder offsets
  // make the front left to back left, back left to back right, back right to front right, front right to front left
  // Add 90 to all encoder offsets
  // Switch robot's lenght to width

  // ROBOT WIDTH AND LENGHT
  // TODO: check the following values
  public static final double DRIVETRAIN_WHEELBASE_WIDTH =  26.5 / Constants.INCHES_PER_METER;
  public static final double DRIVETRAIN_WHEELBASE_LENGTH = 20.5 / Constants.INCHES_PER_METER; // 20.5 for batery_loc

  // SWERVE MODULE STATES
  public static final int FRONT_LEFT_MODULE_STATE_INDEX = 0;
  public static final int FRONT_RIGHT_MODULE_STATE_INDEX = 1;
  public static final int BACK_LEFT_MODULE_STATE_INDEX = 2;
  public static final int BACK_RIGHT_MODULE_STATE_INDEX = 3;

  // FRONT LEFT
  public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 7; //1
  public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER = 0; //1
  public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 6; //2
  public static final double FRONT_LEFT_ANGLE_OFFSET_COMPETITION = Math.toRadians(-100+180); //3.0346

  // FRONT RIGHT
  public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 1; //7
  public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER = 2; //0
  public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 0; //8
  public static final double FRONT_RIGHT_ANGLE_OFFSET_COMPETITION = Math.toRadians(146+180); //2.9835

  // BACK LEFT
  public static final int DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 5; //3
  public static final int DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER = 1; //3
  public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 4; //10
  public static final double BACK_LEFT_ANGLE_OFFSET_COMPETITION = Math.toRadians(-13.2); // 3.0775

  // BACK RIGHT
  public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 3; //5
  public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER = 3;//2
  public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 2; //6
  public static final double BACK_RIGHT_ANGLE_OFFSET_COMPETITION = Math.toRadians(-95.3); //3.01

  //Elevator Constants
  public static final int ElEVATOR_MOTOR_ID = 9;
  public static final int INNER_ELEVATOR_ENCODER_A = 0;
  public static final int INNER_ELEVATOR_ENCODER_B = 1;

  public static final int OUTER_ELEVATOR_ENCODER_A = 8;
  public static final int OUTER_ELEVATOR_ENCODER_B = 9;

  public static class ElevatorState {
    public double ELEVATOR_POSITION;
    public double CORAL_ARM_ANGLE;
    public double ALGAE_ARM_ANGLE;

    ElevatorState(double elevatorPosition, double coralArmAngle, double algaeArmAngle) {
      ELEVATOR_POSITION = elevatorPosition;
      CORAL_ARM_ANGLE = coralArmAngle;
      ALGAE_ARM_ANGLE = algaeArmAngle;
    }
  }

  public static final ElevatorState E_STATE_ALGAE_PICKUP = new Constants.ElevatorState(0, 0, 0);
  public static final ElevatorState E_STATE_ALGAE_LOW = new Constants.ElevatorState(0, 0, 0);
  public static final ElevatorState E_STATE_ALGAE_HIGH = new Constants.ElevatorState(0, 0, 0);
  public static final ElevatorState E_STATE_ALGAE_PROCESSOR = new Constants.ElevatorState(0, 0, 0);

  //TODO: test these values
  public static final ElevatorState E_STATE_CORAL_SOURCE = new Constants.ElevatorState(0, -45, -55);
  public static final ElevatorState E_STATE_CORAL_REEF_1 = new Constants.ElevatorState(0, -90, -90); // TODO: figure out coralArmAngle
  public static final ElevatorState E_STATE_CORAL_REEF_2 = new Constants.ElevatorState(0, -90, -55);
  public static final ElevatorState E_STATE_CORAL_REEF_3 = new Constants.ElevatorState(0, -90, -55);
  public static final ElevatorState E_STATE_CORAL_REEF_4 = new Constants.ElevatorState(0, -130, -55); // TODO: find maximum angle for the coralArmAngle

  /*
   * E_STATE_ALGAE_LOW
   * E_STATE_ALGAE_HIGH
   * E_STATE_ALGAE_PROCESSOR
   * 
   * E_STATE_CORAL_SOURCE
   * E_STATE_CORAL_REEF_1
   * E_STATE_CORAL_REEF_2
   * E_STATE_CORAL_REEF_3
   * E_STATE_CORAL_REEF_4
   */
  // INTAKE CONSTANTS
  // Algae
  public static final int ALGAE_LEFT_INTAKE_MOTOR = 6;
  public static final int ALGAE_RIGHT_INTAKE_MOTOR = 5;
  public static final double ALGAE_INTAKE_ROTATION_SPEED = 0.3;
  // voltage values for algae intake
  public static final double ALGAE_INTAKE_VOLTAGE_OUT = -0.5;
  public static final double ALGAE_INTAKE_VOLTAGE_IN = 1.5;
  public static final double ALGAE_INTAKE_VOLTAGE_HOLD = 0.5;

  public static final int ALGAE_ARM_MOTOR = 4;
  public static final double CORAL_ARM_ENCODER_OFFSET = 0;//15.309518814086914;

  // CORAL
  public static final int CORAL_LEFT_INTAKE_MOTOR = 3;
  public static final int CORAL_RIGHT_INTAKE_MOTOR = 2;
  // voltage values for coral intake
  public static final double CORAL_INTAKE_VOLTAGE_OUT = -3;
  public static final double CORAL_INTAKE_VOLTAGE_IN = 3;

  public static final int CORAL_ARM_MOTOR = 1;

  // hat constants 
  public static final int HAT_JOYSTICK_TRIM_POSITION = RIGHT_JOYSTICK_CHANNEL;
  public static final int HAT_JOYSTICK_TRIM_ROTATION = LEFT_JOYSTICK_CHANNEL;
  public static final double HAT_POWER_MOVE = 0.1;
  public static final double HAT_POWER_ROTATE = 0.3;
  // Hat trim target speed is 15 degrees per second
  // One time step is 0.02 seconds
  // 0.3 degrees per time step is our target change when the hat is active
  public static final double HAT_POSE_TARGET_PER_TIME_STEP = -0.3; // negative is raising the arm
  
  public static final int HAT_POV_MOVE_LEFT = 270;
  public static final int HAT_POV_MOVE_RIGHT = 90;
  public static final int HAT_POV_MOVE_FORWARD = 0;
  public static final int HAT_POV_MOVE_BACK = 180;
  public static final int HAT_POV_0 = 0; // Left hat up
  public static final int HAT_POV_180 = 180; // Left hat down
  public static final int HAT_POV_ROTATE_LEFT = 270;
  public static final int HAT_POV_ROTATE_RIGHT = 90;
  /*Changed max speed from 2pi rad/s since max no load speed is 15 ft/s and with our robot
  the radius from the center of our robot to the center of the module is 16.5 inches therefore
  with a max commanded angular speed of 2pi radians our max commanded speed is 8.6 ft/s which
  is about half the theoretical maximum. We don't want to go to the absolute maximum which would
  approximately be 4pi rad/s so we choose 3pi rad/s as the maximum.
  Note: if we run into problems during autos with it lagging, we may want to revert it back to 2pi rad/s. */
  public static final double kMaxModuleAngularSpeedRadiansPerSecond = 3 * Math.PI; 
  //We set it to 9pi rad/s^2 so that we get to the maximum speed within 1/3 of a second
  //We may want to back off on this if autos start lagging
  public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 12 * Math.PI;

  public static final double maxModuleLinearSpeed = 1.75;
  public static final double maxModuleLinearAccelaration = 8;
  // public static final double maxModuleLinearSpeed = 3.5;
  // public static final double maxModuleLinearAccelaration = 16;

  // 6.7

  public static final int kEncoderCPR = 4096; // kraken encoder ticks per revolution
  public static final double kWheelDiameterMeters = 4.0 / 39.37;
  // Divided by 100 to convert cm to m
  public static final double kCalibrationFactor = 2.48 * 0.95 *1.03 / 100; // we tested the actual traveled distance vs the distance the robot thinks it traveled to come up with this factor
  public static final double kDriveEncoderDistancePerPulse =
      // Assumes the encoders are directly mounted on the wheel shafts
      // (kWheelDiameterMeters * Math.PI) * (1.0 / (45.0 / 15.0) / (17.0 / 27.0) / (50.0 / 14.0));  //Mark 4i L2 Gear Ratio // should be 5.14 to 1 four L4 on BunnyBot
      (kWheelDiameterMeters * Math.PI) * 6.7 * kCalibrationFactor;  //Mark 4i L2 Gear Ratio // should be 5.14 to 1 four L4 on BunnyBot
      // (kWheelDiameterMeters * Math.PI) * 5.14 * kCalibrationFactor;  //Mark 4i L2 Gear Ratio // should be 5.14 to 1 four L4 on BunnyBot
      // 0.0014785364645989762;

      // Some additional notes... 
      //
      //6.74603174603 gear ratio for eclipse...
      // 42 counts per revolution of the motor
      // 6.746 * 42 counts per revolution of the wheel
      // 283.333333333 counts per revolution of the wheel
      // (kWheelDiameterMeters * Math.PI) circumference in meters
      // (kWheelDiameterMeters * Math.PI) / 283.3333  meters per count i.e. distance per pulse
      // 0.0011265396 meters per pulse for eclipse
      //
      //5.14 L4 gear ratio for bunnybot
      //42 counts per revolution of the motor
      //5.14 * 42 counts per revolution of the wheel
      //215.88 counts per revolution of the wheel
      // (kWheelDiameterMeters * Math.PI) circumference in meters
      // (kWheelDiameterMeters * Math.PI)/ 215.88 meters per count i.e. distance per pulse
      // 0.0014785364645989762 meters per pulse for bunnybot2024

  // put into manual mode, manually read position and rotate wheel

  public static final double kTurningEncoderDistancePerPulse =
      // Assumes the encoders are on a 1:1 reduction with the module shaft.
      (2 * Math.PI) / (double) kEncoderCPR;

  public static final double kPModuleTurningController = 0.5;

  public static final double kPModuleDriveController = 0;

  public static final int kDriverControllerPort = 0;
  public static final int kOperatorControllerPort = 1;

  // VISION CONSTANTS
  public static double OBJECT_DETECTION_LATENCY = 0.217; // seconds
  public static final double VISION_NOTE_CAM_TILT = Units.degreesToRadians(0);
  public static final double VISION_APRIL_TAG_PRO_TILT = Units.degreesToRadians(0);

  // {x, y} in inches, +y is robots front, +x is to the right
  public static final double[] VISION_TOTE_CAM_OFFSET = {-Units.metersToInches((DRIVETRAIN_WHEELBASE_WIDTH/2)) + 4, -Units.metersToInches(DRIVETRAIN_WHEELBASE_LENGTH/2)}; // TODO: calculate these values based from the center of the robot
  public static final double[] VISION_BALLOON_CAM_OFFSET = {};

  public static final int TARGET_TRIGGER_DISTANCE_APRIL_TAG = 0;

  // STREAMDECK CONSTANTS
  public static final String SD_SOURCE = "0";
  public static final String SD_REEF_LEVEL_1 = "1";
  public static final String SD_REEF_LEVEL_2 = "2";
  public static final String SD_REEF_LEVEL_3 = "3";
  public static final String SD_REEF_LEVEL_4 = "4";

  public static final String SD_REEF_POS_1 = "1";
  public static final String SD_REEF_POS_2 = "2";
  public static final String SD_REEF_POS_3 = "3";
  public static final String SD_REEF_POS_4 = "4";
  public static final String SD_REEF_POS_5 = "5";
  public static final String SD_REEF_POS_6 = "6";


  public static final String SD_REEF_LEFT = "L";
  public static final String SD_REEF_RIGHT = "R";

  // AUTO CONSTANTS
  
  // AUTOMOVESWERVE CONSTANTS
  public static final double CHANGE_IN_X_PER_SECOND= 0.714;
  public static final double CHANGE_IN_Y_PER_SECOND= 0.717;

  // Conversions
  public static final double FEET_TO_METERS = 3.281;

  //BUTTON BINDINGS

  // right buttons
  public static final int CORAL_INTAKE_OUT_BUTTON = 1;
  public static final int SWERVE_RESET_BUTTON = 9;
  public static final int ZERO_ODOMETRY_BUTTON = 11;
  public static final int ELEVATOR_FIRST_STAGE_UP_BUTTON = 5;
  public static final int ELEVATOR_FIRST_STAGE_DOWN_BUTTON = 3;
  public static final int ELEVATOR_SECOND_STAGE_UP_BUTTON = 6;
  public static final int ELEVATOR_SECOND_STAGE_DOWN_BUTTON = 4;
  public static final int ELEVATOR_ZERO_POWER_BUTTON = 2;


  // left buttons
  //TODO: change these to better buttons
  public static final int CORAL_INTAKE_IN_BUTTON = 1;
  public static final int ALGAE_INTAKE_IN_BUTTON = 5;
  public static final int ALGAE_INTAKE_OUT_BUTTON = 6;


}
