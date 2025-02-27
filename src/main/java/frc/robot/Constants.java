// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import swervelib.math.Matter;

import frc.robot.commands.swervedrive.auto.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }


  public final class Elevator
  {
    public static final int ELEVATOR_MOTOR_ID = 40;

   
    public static final double BASE_STAGE_ENCODER_VALUE = 0;
    public static final double FIRST_CORAL_STAGE_ENCODER_VALUE = 10;
    public static final double SECOND_CORAL_STAGE_ENCODER_VALUE = 20;
    public static final double THIRD_CORAL_STAGE_ENCODER_VALUE = 30;
    public static final double FOURTH_CORAL_STAGE_ENCODER_VALUE = 40;
    public static final double COLLECT_CORAL_STAGE_ENCODER_VALUE = 40;

    public static final double PID_P = 0;
    public static final double PID_I = 0;
    public static final double PID_D = 0;
    public static final double PID_IZ = 0;
    public static final double PID_F = 0;
    public static final double POTENTIOMETER_MOVEMENT_SPEED = 0.2;
    public static final double HOMING_SPEED = -.2;
    public static final int LIMIT_SWITCH_PWM_PORT = 0;

    public final class Potentiometer
    {
      public static final int PWM_PORT = 0;
      public static final double LOWER_BOUND = 0.0;
      public static final double UPPER_BOUND = 1.0;
      public static final double MOVEMENT_TOLERATION = 0.05;

      public static final double BASE_STAGE_POT_VALUE = 0.0;
      public static final double FIRST_CORAL_STAGE_POT_VALUE = 0.0;
      public static final double SECOND_CORAL_STAGE_POT_VALUE = 0.0;
      public static final double THIRD_CORAL_STAGE_POT_VALUE = 0.0;
      public static final double FOURTH_CORAL_STAGE_POT_VALUE = 0.0;
      public static final double COLLECT_CORAL_STAGE_POT_VALUE = 0.0;
    }
  }

  public final class Lift
  {
    public static final int LIFT_MOTOR_ID = 50;
    public static final double START_STAGE_ENCODER_VALUE = 25;
    public static final double CATCH_STAGE_ENCODER_VALUE = 50;
    public static final double LIFT_STAGE_ENCODER_VALUE = 0;

    public static final double PID_P = 0;
    public static final double PID_I = 0;
    public static final double PID_D = 0;
    public static final double PID_IZ = 0;
    public static final double PID_F = 0;
  }

  public final class Coral
  {

    public static final double DISTANCE_FROM_CENTER = 0.2794; //In Meters

    public final class RotationMotor 
    {
      public static final int CORAL_MOTOR_ID = 60;
      public static final double START_POSITION_ENCODER_VALUE = 25;
      public static final double COLLECT_POSITION_ENCODER_VALUE = 50;
      public static final double SCORE_LOWER_POSITION_ENCODER_VALUE = 0;
      public static final double SCORE_TOP_POSITION_ENCODER_VALUE = 0;
      public static final double PID_P = 0;
      public static final double PID_I = 0;
      public static final double PID_D = 0;
      public static final double PID_IZ = 0;
      public static final double PID_F = 0;
    }

    public final class Servo
    {
      public static final int SERVO_PWM_PORT = 1;
      public static final double DOOR_CLOSED_ANGLE = 0.0;
      public static final double DOOR_OPEN_ANGLE = 180.0;
    }

    public final class WheelMotor 
    {
      public static final int WHEEL_MOTOR_ID = 61;
      public static final double INTAKE_SPEED = .1;
      public static final double OUTPUT_SPEED = -.2;
      public static final double OUTPUT_DELAY = 3; //In Seconds

      public static final int TOF_SENSOR = 62;  //To change the ID for the TOF Sensor, drop the code and enable the bot, and then go to http://10.28.90.2:5812/ in a browser.
      public static final int TOF_TRIGGER_DIST = 10; //In Millimeters
    }
    
  }

  

  public static class ShuffleboardConstants {

    //Shuffleboard Constants
     public static final String UNIVERSAL_MODE_CHOICE = "allAutos";  //Choices for what Autos to load. Valid Choices: competiton, testing, allAutos
  }

  public static class LimeLight{

    public static final String LIMELIGHT_NAME = "limelight";

    //View For AprilTag Labeling: Red Barge Is On Bottom
    public static class BlueAprilTags{
      //Coral Stations Tags
      public static final int UPPER_CORAL_STATION = 13;
      public static final int LOWER_CORAL_STATION = 12;

      //Reef Tags, Labled As Clock Positions
      public static final int REEF_ONE_POSITION = 20;    //Branches J & I
      public static final int REEF_THREE_POSITION = 21;  //Branches H & G
      public static final int REEF_FIVE_POSITION = 22;   //Branches F & E
      public static final int REEF_SEVEN_POSITION = 17;  //Branches D & C
      public static final int REEF_NINE_POSITION = 18;   //Branches B & A
      public static final int REEF_ELEVEN_POSITION = 19; //Branches L & K
      public static final int[] REEF_APRILTAGS = {17,18,19,20,21,22};

      //Barge Tags
      public static final int BLUE_BARGE = 14;
      public static final int RED_BARGE = 15;

      //Processor Tags
      public static final int PROCESSOR = 16;
    }
    
    //View For AprilTag Labeling: Red Barge Is On Bottom
    public static class RedAprilTags{
      //Coral Stations Tags
      public static final int UPPER_CORAL_STATION = 2;
      public static final int LOWER_CORAL_STATION = 1;

      //Reef Tags, Labled As Clock Positions
      public static final int REEF_ONE_POSITION = 8;     //Branches D & C
      public static final int REEF_THREE_POSITION = 7;   //Branches B & A
      public static final int REEF_FIVE_POSITION = 6;    //Branches L & K
      public static final int REEF_SEVEN_POSITION = 11;  //Branches J & I
      public static final int REEF_NINE_POSITION = 10;   //Branches H & G
      public static final int REEF_ELEVEN_POSITION = 9;  //Branches F & E
      public static final int[] REEF_APRILTAGS = {7,8,9,10,11};

      //Barge Tags
      public static final int BLUE_BARGE = 4;
      public static final int RED_BARGE = 5;

      //Processor Tags
      public static final int PROCESSOR = 3;
    }

    public static class BlueReefPositions{
      public static class CoralPoses{
        // FMS Branch Names
        public static final Pose2d A = new Pose2d();
        public static final Pose2d B = new Pose2d();
        public static final Pose2d C = new Pose2d();
        public static final Pose2d D = new Pose2d();
        public static final Pose2d E = new Pose2d();
        public static final Pose2d F = new Pose2d();
        public static final Pose2d G = new Pose2d();
        public static final Pose2d H = new Pose2d();
        public static final Pose2d I = new Pose2d();
        public static final Pose2d J = new Pose2d();
        public static final Pose2d K = new Pose2d();
        public static final Pose2d L = new Pose2d();
      }
      public static class AlgaePoses{
        // Clock Positions Relative To Scoring Table (6 o'clock)
        public static final Pose2d ONE = new Pose2d();
        public static final Pose2d THREE = new Pose2d();
        public static final Pose2d FIVE = new Pose2d();
        public static final Pose2d SEVEN = new Pose2d();
        public static final Pose2d NINE = new Pose2d();
        public static final Pose2d ELEVEN = new Pose2d();
      }
    }

    public static class RedReefPositions{
      public static class CoralPoses{
        // FMS Branch Names
        public static final Pose2d A = new Pose2d();
        public static final Pose2d B = new Pose2d();
        public static final Pose2d C = new Pose2d();
        public static final Pose2d D = new Pose2d();
        public static final Pose2d E = new Pose2d();
        public static final Pose2d F = new Pose2d();
        public static final Pose2d G = new Pose2d();
        public static final Pose2d H = new Pose2d();
        public static final Pose2d I = new Pose2d();
        public static final Pose2d J = new Pose2d();
        public static final Pose2d K = new Pose2d();
        public static final Pose2d L = new Pose2d();
      }
      public static class AlgaePoses{
        // Clock Positions Relative To Scoring Table (6 o'clock)
        public static final Pose2d ONE = new Pose2d();
        public static final Pose2d THREE = new Pose2d();
        public static final Pose2d FIVE = new Pose2d();
        public static final Pose2d SEVEN = new Pose2d();
        public static final Pose2d NINE = new Pose2d();
        public static final Pose2d ELEVEN = new Pose2d();
      }
    }

  }
}