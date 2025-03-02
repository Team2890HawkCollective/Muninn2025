// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
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

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(14.5);
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
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;

    public static final int JOYSTICK_X_CHANNEL = 0;
    public static final int JOYSTICK_Y_CHANNEL = 1;
    public static final double JOYSTICK_IS_LEFT = 1.00;
    public static final double JOYSTICK_IS_RIGHT = -1.00;
    public static final double JOYSTICK_IS_UP = 1.00;
    public static final double JOYSTICK_IS_DOWN = -1.00;
  }

  public final class Elevator {
    public static final int ELEVATOR_MOTOR_ID = 40;

    public static final double DEADZONE = 0.1;

    public static final double ELEVATOR_UP_SPEED = 0.75;
    
    public static final int CORAL_STAGE_BASE = 0;
    public static final int CORAL_STAGE_L1 = 1;
    public static final int CORAL_STAGE_L2 = 2;
    public static final int CORAL_STAGE_L3 = 3;
    public static final int CORAL_STAGE_L4 = 4;
    public static final int ALGAE_STAGE_L2 = 5;
    public static final int ALGAE_STAGE_L3 = 6;

    public static double BASE_STAGE_ENCODER_VALUE = 0;
    public static final double L1_CORAL_STAGE_ENCODER_DIFFERENCE = 10;
    public static final double L2_CORAL_STAGE_ENCODER_DIFFERENCE = 0.251623213291168; // -2.058275461196899
    public static final double L3_CORAL_STAGE_ENCODER_DIFFERENCE = -4.001606464385986;
    public static final double L4_CORAL_STAGE_ENCODER_DIFFERENCE = 50;
    public static final double L2_ALGAE_STAGE_ENCODER_DIFFERENCE = 0.251623213291168; // Get Values
    public static final double L3_ALGAE_STAGE_ENCODER_DIFFERENCE = -4.001606464385986; // Get Values

    public static final double[] STAGE_ENCODER_DIFFERENCES = {
      BASE_STAGE_ENCODER_VALUE,
      L1_CORAL_STAGE_ENCODER_DIFFERENCE,
      L2_CORAL_STAGE_ENCODER_DIFFERENCE,
      L3_CORAL_STAGE_ENCODER_DIFFERENCE,
      L4_CORAL_STAGE_ENCODER_DIFFERENCE,
      L2_ALGAE_STAGE_ENCODER_DIFFERENCE,
      L3_ALGAE_STAGE_ENCODER_DIFFERENCE
    };

    public static final double PID_P = 0.05;
    public static final double PID_I = 0;
    public static final double PID_D = 0;
    public static final double PID_IZ = 0;
    public static final double PID_F = 0;
    public static final double POTENTIOMETER_MOVEMENT_SPEED = 0.2;
    public static final double HOMING_SPEED = -0.25;
    public static final int LIMIT_SWITCH_PWM_PORT = 1;

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
      //public static final int SERVO_PWN_PORT = ;
      public static final double R_UNLOCK_ANGLE = 0.3;
      public static final double R_LOCK_ANGLE = 0.15;
    }
   
  }

  public final class Coral {
    public final class RotationMotor {
      public static final int CORAL_MOTOR_ID = 60;
      public static final double START_POSITION_ENCODER_VALUE = 0.0;
      public static final double SCORE_POSITION_ENCODER_VALUE = -19.618974685668945;
      // public static final double SCORE_POSITION_ENCODER_VALUE =
      // -17.214284896850586;
      public static final double PID_P = 0.075;
      public static final double PID_I = 0;
      public static final double PID_D = 0.01;
      public static final double PID_IZ = 0;
      public static final double PID_F = 0;
    }

    public final class CoralServo {
      public static final int SERVO_PWM_PORT = 9;
      public static final double DOOR_CLOSED_ANGLE = 40.0;
      public static final double DOOR_OPEN_ANGLE = 100.0;
      public static final long OUTPUT_DELAY = 3000;
    }

    public static final int TOF_SENSOR = 62; // To change the ID for the TOF Sensor, drop the code and enable the bot,
                                               // and then go to http://10.28.90.2:5812/ in a browser.
    public static final int TOF_TRIGGER_DIST = 10; // In Millimeters

  }

  public final class Algae {
    public final class Rotation {
      public static final int ALGAE_ROTATION_MOTOR_ID = 61;
      public static final double START_POSITION_ENCODER_VALUE = -22;
      public static final double COLLECT_ENCODER_VALUE_POS = 60.31047821044922;
      public static final double CARRY_ENCODER_VALUE = 4;
      public static final double PID_P = 0.1;
      public static final double PID_I = 0;
      public static final double PID_D = 0;
      public static final double PID_IZ = 0;
      public static final double PID_F = 0;

      public static final double DEADZONE = 0.1;

    }

    public final class Wheel {
      public static final int ALGAE_WHEEL_MOTOR_ID = 62;
      public static final double WHEEL_INTAKE_SPEED = -0.35;
      public static final double WHEEL_OUTPUT_SPEED = 0.5;

      //public static final int LIMIT_SWITCH_PORT = 3;
      public static final int TOF_SENSOR = 61;
      public static final int TOF_DISTANCE = 200; // In Millimeters

    }

  }

  public static class ShuffleboardConstants {

    // Shuffleboard Constants
    public static final String UNIVERSAL_MODE_CHOICE = "allAutos"; // Choices for what Autos to load. Valid Choices:
                                                                   // competiton, testing, allAutos
  }

  public static class LED {
    public static final int SIGNAL_LIGHTS_PORT = 4;
    public static final int SIGNAL_LIGHTS_LENGTH = 20;
  }

}