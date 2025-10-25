// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import swervelib.math.Matter;

import frc.robot.commands.swervedrive.auto.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = 48.1442941; // 106.14 Lbs
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(10);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  // public static final class AutonConstants
  // {
  //
  // public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0,
  // 0);
  // public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
  // }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.08;
    public static final double LEFT_Y_DEADBAND = 0.3;
    public static final double RIGHT_X_DEADBAND = 0.3;
    public static final double TURN_CONSTANT = 6;

    public static final int JOYSTICK_X_CHANNEL = 0;
    public static final int JOYSTICK_Y_CHANNEL = 1;
    public static final double JOYSTICK_IS_LEFT = 1.00;
    public static final double JOYSTICK_IS_RIGHT = -1.00;
    public static final double JOYSTICK_IS_UP = 1.00;
    public static final double JOYSTICK_IS_DOWN = -1.00;
  }

  public final class Elevator {
    public static final int ELEVATOR_MOTOR1_ID = 30;
    public static final int ELEVATOR_MOTOR2_ID = 41;

    public static final double DEADZONE = 0.1;

    public static final double ELEVATOR_UP_SPEED = -0.45;
    public static final double HOMING_SPEED = 0.35;
    public static final double kDt = 0.02;

   public static final int CORAL_STAGE_BASE = 0;
    public static final int CORAL_STAGE_L1 = 1;
    public static final int CORAL_STAGE_L2 = 2;
    public static final int CORAL_STAGE_L3 = 3;
    public static final int CORAL_STAGE_L4 = 4;
    public static final int ALGAE_STAGE_L2 = 5;
    public static final int ALGAE_STAGE_L3 = 6;

    public static double BASE_STAGE_ENCODER_VALUE = 0.568913459777832;
    public static final double L1_CORAL_STAGE_ENCODER_DIFFERENCE = 10;
    public static final double L2_CORAL_STAGE_ENCODER_DIFFERENCE = -29; //-28.505441665649414; // Old Number  4.2342
    public static final double L3_CORAL_STAGE_ENCODER_DIFFERENCE = -42.540; //-46.79143524169922; // Old Number 6.3520
    public static final double L4_CORAL_STAGE_ENCODER_DIFFERENCE = -64.5; // Old Number 63.65
    public static final double L2_ALGAE_STAGE_ENCODER_DIFFERENCE = -12.136439323425293; // Get Values
    public static final double L3_ALGAE_STAGE_ENCODER_DIFFERENCE = -26.594308853149414; // Get Values

    public static final double[] STAGE_ENCODER_DIFFERENCES = {
        BASE_STAGE_ENCODER_VALUE,
        L1_CORAL_STAGE_ENCODER_DIFFERENCE,
        L2_CORAL_STAGE_ENCODER_DIFFERENCE,
        L3_CORAL_STAGE_ENCODER_DIFFERENCE,
        L4_CORAL_STAGE_ENCODER_DIFFERENCE,
        L2_ALGAE_STAGE_ENCODER_DIFFERENCE,
        L3_ALGAE_STAGE_ENCODER_DIFFERENCE
    }; 

    public static final double PID_P = 0.3;
    public static final double PID_I = 0;
    public static final double PID_D = 0.4;
    public static final double PID_IZ = 0;
    public static final double PID_F = 0;
    public static final double POTENTIOMETER_MOVEMENT_SPEED = 0.2;
    public static final int LIMIT_SWITCH_PWM_PORT = 1;

    public final class TrapezoidProfile{
      public static final double kRelativeEncoderScaleRevToMeters = 0.0315;
      public static final double kAbsoluteEncoderScaleVoltsToMeters = 0.498;
      public static final double kAbsoluteEncoderOffsetVoltsToMeters = 0.43;
      
      public static final double kP = 6;
      public static final double kI = 0;
      public static final double kD = 0.46;

      //get from SYSID
      public static final double kS = 0.04435;
      public static final double kG = 0.117;
      public static final double kV = 0.117;
      public static final double kA = 0.00803;

      public static final double elevatorHomeHeightMeters = Units.inchesToMeters(0);  // only valid when elevator is homed;

      public static final double kToleranceMeters = Units.inchesToMeters(1.0);

      public static final int kElevatorCurrentLimit = 60;

      public static final double kElevatorMaxHeightMeters = Units.inchesToMeters(73);
      public static final double kElevatorSpeedSafeHeightMeters = Units.inchesToMeters(40);
      public static final double kElevatorMinHeightMeters = Units.inchesToMeters(17.5);

      public static final double kL1CoralHeightMeters = Units.inchesToMeters(21);
      public static final double kL2CoralHeightMeters = Units.inchesToMeters(31);
      public static final double kL3CoralHeightMeters = Units.inchesToMeters(46);
      public static final double kL4CoralHeightMeters = Units.inchesToMeters(70);
      
      public static final double kL1AlgaeHeightMeters = Units.inchesToMeters(24);
      public static final double kL2AlgaeHeightMeters = Units.inchesToMeters(39);
      public static final double kL3AlgaeHeightMeters = Units.inchesToMeters(54);  

      public static final double kL4AlgaeWindupHeightMeters = Units.inchesToMeters(70.5);  

      public static final double kSafeHomeHeightMeters = Units.inchesToMeters(0);

      //public static final double kElevatorMaxVelocityMPS = 2.0;  // MPS
		  public static final double kElevatorMaxAccelerationMPSPS = 4.0; // MPSS  was 6

      public static final double kElevatorEncoderPositionConversionFactor = kRelativeEncoderScaleRevToMeters; 
      public static final double kElevatorEncoderVelocityConversionFactor = kRelativeEncoderScaleRevToMeters; 
      
      
    }


  }

  public final class Lift {
    public static final int LIFT_MOTOR_ID = 50;
    public static final double START_STAGE_ENCODER_VALUE = 25;
    public static final double CATCH_STAGE_ENCODER_VALUE = 50;
    public static final double LIFT_STAGE_ENCODER_VALUE = 0;

    public static final double PID_P = 0;
    public static final double PID_I = 0;
    public static final double PID_D = 0;
    public static final double PID_IZ = 0;
    public static final double PID_F = 0;

    public final class LIFT_SERVO {
      public static final int SERVO_PWN_PORT = 0;
      public static final double R_UNLOCK_ANGLE = 0.3;
      public static final double R_LOCK_ANGLE = 0.15;
    }

  }

  public final class Coral {

    public static final double DISTANCE_FROM_CENTER = 0.2794; // In Meters (11 Inches)

    // Branches are 13 Inches apart, so 6.5 each from the center.
    public static final double LEFT_BRANCH_OFFSET = Units.inchesToMeters(-17.5); // In Inches
    public static final double RIGHT_BRANCH_OFFSET = Units.inchesToMeters(-4.25); // In Inches
    public static final double LAUNCH_CORAL_SPEED = -.7;


    public final class RotationMotor {
      public static final int CORAL_MOTOR_ID = 39;

      public static final double START_POSITION_ENCODER_VALUE = 0.0; //-1.023809194564819; // This tries to not kill the ramp
      public static final double SCORE_POSITION_ENCODER_VALUE = -15.00000286102295;
      public static final double L4_POSITION_ENCODER_VALUE = -17.0;
      // public static final double SCORE_POSITION_ENCODER_VALUE =
      // -17.214284896850586;
      public static final double PID_P = 0.05;
      public static final double PID_I = 0;
      public static final double PID_D = 0.03;
      public static final double PID_IZ = 0;
      public static final double PID_F = 0;
      public static final double ROTATE_DELAY = 1.25;
    }

    public final class CoralServo {
      public static final int SERVO_PWM_PORT = 9;
      public static final double DOOR_OPEN_ANGLE = 40.0;
      public static final double DOOR_CLOSED_ANGLE = 120.0;
      public static final long OUTPUT_DELAY = 3000;
    }
    
    public static final double INTAKE_WHEEL_SPEED = .2;
    public static final double OUTPUT_WHEEL_SPEED = -1;
    public static final int INBOARD_SENSOR = 62; // To change the ID for the TOF Sensor, drop the code and enable the bot,
    public static final int OUTBOARD_SENSORY = 63; // and then go to http://10.28.90.2:5812/ in a browser.
    public static final int TOF_TRIGGER_DIST = 10; // In Millimeters

  }

  public final class Algae {

    public static final double OFFSET = Units.inchesToMeters(0.0); // In Inches

    public final class Rotation {
      public static final double MANUAL_SPEED = .60;
      public static final int ALGAE_ROTATION_MOTOR_ID = 0;
      public static final double START_POSITION_ENCODER_VALUE = 0;
      public static final double COLLECT_ENCODER_VALUE_POS = 73.09744262695312;
      public static final double CARRY_ENCODER_VALUE = 24.571;
      public static final double PROCESSOR_ENCODER_VALUE = 57.50067901611328;
      public static final double LIFT_POSITION_ENCODER_VALUE = 19.952302932739258;

      //new algae encoders for new arm
      public static final double GROUND_PICKUP_ALGAE_ENCODER_VALUE = -21.38085174560547;
      public static final double PROCESSOR_ALGAE_ENCODER_VALUE = -16.59521484375;
      public static final double COLLECT_ALGAE_ENCODER_VALUE = -40.880531311035156;




      public static final double PID_P = 0.05;
      public static final double PID_I = 0;
      public static final double PID_D = 0.1;
      public static final double PID_IZ = 0;
      public static final double PID_F = 0;

      public static final double DEADZONE = 0.1;

    }

    public final class Wheel {
      public static final int ALGAE_WHEEL_MOTOR_ID = 62;
      public static final double WHEEL_INTAKE_SPEED = -1.0;
      public static final double WHEEL_OUTPUT_SPEED = 1.0;
      public static final double INTAKEN_ALGAE_WHEEL_CURRENT = 60;


      // public static final int LIMIT_SWITCH_PORT = 3;
      //public static final int TOF_SENSOR = 61;
      //public static final int TOF_DISTANCE = 200; // In Millimeters

    }

  }

  public static class ShuffleboardConstants {

    // Shuffleboard Constants
    public static final String UNIVERSAL_MODE_CHOICE = "allAutos"; // Choices for what Autos to load. Valid Choices:
                                                                   // competiton, testing, allAutos
    public static final String CONTROL_MODE = "manual"; // manual or buttonboard
  }

  public static class LimeLight {

    public static final String LIMELIGHT_NAME = "limelight";

    public static final int[] ALL_REEF_APRILTAGS = { 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22 };

    public static final AprilTagFieldLayout APRILTAG_FIELD_LAYOUT = AprilTagFieldLayout
      .loadField(AprilTagFields.k2025ReefscapeAndyMark);

    public static final double BUMPER_WIDTH = Units.inchesToMeters(0.0); // Get This Value // Original: 2.75
    //public static final double ROBOT_WIDTH = Units.inchesToMeters(30 + BUMPER_WIDTH); // Tis a square, don't need this
    public static final double ROBOT_SIDE_LENGTH = Units.inchesToMeters(29);
    public static final Transform2d HALF_ROBOT = new Transform2d(ROBOT_SIDE_LENGTH / 3.0, 0, new Rotation2d());
  }

  public static class LED {
    public static final int SIGNAL_LIGHTS_PORT = 4;
    public static final int SIGNAL_LIGHTS_LENGTH = 256;
    public static final int END_GAME_TIME_START = 20;
  }
}
