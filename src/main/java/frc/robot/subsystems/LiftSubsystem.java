package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.Lift;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LiftSubsystem extends SubsystemBase {

  private static TalonFX liftMotor = new TalonFX(Constants.Lift.LIFT_MOTOR_ID);
  private static Servo liftRatchet = new Servo(Constants.Lift.LIFT_SERVO.SERVO_PWN_PORT);

  public LiftSubsystem() {
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = Constants.Lift.PID_P;
    slot0Configs.kI = Constants.Lift.PID_I;
    slot0Configs.kD = Constants.Lift.PID_D;
    liftMotor.getConfigurator().apply(slot0Configs);
  }

  public void periodic() {
    // This method will be called once per scheduler
    //SmartDashboard.putData("Lift Voltage Rotations", (Sendable) liftMotor.getPosition());
  }

  public Command moveToPositionCommand(PositionVoltage positionVoltage) {
    return runOnce(() -> moveToPosition(positionVoltage));
  }

  public void moveToPosition(PositionVoltage positionVoltage) {
    liftMotor.setControl(positionVoltage);
  }

  public Command moveToCatchPositionCommand() {
    return run(() -> catchPosition());
  }

  public void catchPosition() {
    liftMotor.set(-.1);
  }

  public Command moveToStartPositionCommand() {
    return run(() -> startPosition());
  }

  public void startPosition() {
    liftMotor.set(.1);
  }

  public Command stopLiftMotorCommand() {
    return runOnce(() -> stopLiftMotor());
  }
  public void stopLiftMotor() {
    liftMotor.set(0);
  }

  public Command retractRatchetCommand() {
    return runOnce(() -> retractRatchet());
  }

  public Command lockRatchetCommand() {
    return runOnce(() -> lockRatchet());
  }

  public void retractRatchet() {
    toggleRatchet(false);
  }

  public void lockRatchet() {
    toggleRatchet(true);
  }

  public static void toggleRatchet(boolean toggle) {
    if (toggle == true) {
      liftRatchet.set(Constants.Lift.LIFT_SERVO.R_LOCK_ANGLE);
    } else
      liftRatchet.set(Constants.Lift.LIFT_SERVO.R_UNLOCK_ANGLE);
  }

}
