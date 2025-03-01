package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.playingwithfusion.TimeOfFlight;

public class CoralSubsystem extends SubsystemBase {
    private static SparkMax coralRotationalMotor = new SparkMax(Constants.Coral.RotationMotor.CORAL_MOTOR_ID,
            MotorType.kBrushless);
    private static SparkMaxConfig coralRotationalPIDConfig = new SparkMaxConfig();
    private static SparkClosedLoopController coralRotationalPIDController;
    // private static SparkPIDController coralRotationalPIDController =
    // coralRotationalMotor.

    // private static SparkMax coralWheelMotor = new
    // SparkMax(Constants.Coral.WheelMotor.WHEEL_MOTOR_ID, MotorType.kBrushless);

    public TimeOfFlight TOFSensor = new TimeOfFlight(Constants.Coral.TOF_SENSOR);
    private static Servo doorServo = new Servo(Constants.Coral.CoralServo.SERVO_PWM_PORT);

    public CoralSubsystem() {
        // coralRotationalPIDConfig.closedLoop.pid(Constants.Coral.RotationMotor.PID_P,
        // Constants.Coral.RotationMotor.PID_I, Constants.Coral.RotationMotor.PID_D);
        coralRotationalPIDConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control. We don't need to pass a closed loop
                // slot, as it will default to slot 0.
                .p(Constants.Coral.RotationMotor.PID_P)
                .i(Constants.Coral.RotationMotor.PID_I)
                .d(Constants.Coral.RotationMotor.PID_D)
                .outputRange(-1, 1)
                // Set PID values for velocity control in slot 1
                .p(0.0001, ClosedLoopSlot.kSlot1)
                .i(0, ClosedLoopSlot.kSlot1)
                .d(0, ClosedLoopSlot.kSlot1)
                .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        coralRotationalMotor.configure(coralRotationalPIDConfig, ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
        coralRotationalPIDController = coralRotationalMotor.getClosedLoopController();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler
        SmartDashboard.putNumber("Coral Relative Encoder", coralRotationalMotor.getEncoder().getPosition());
    }

    public Command rotateToPositionCommand(double encoderValue) {
        return runOnce(() -> rotateToPosition(encoderValue));
    }

    public Command coralIntakeCommand() {
        return rotateToPositionCommand(Constants.Coral.RotationMotor.START_POSITION_ENCODER_VALUE)
                .andThen(runOnce(() -> doorServo.setAngle(Constants.Coral.CoralServo.DOOR_OPEN_ANGLE)))
                // .andThen(runOnce(()->intakeCoral()))
                // .until(() ->{return (TOFSensor.getRange() <
                // Constants.Coral.WheelMotor.TOF_TRIGGER_DIST);})
                // .andThen(()->stopWheels())
                .andThen(runOnce(() -> doorServo.setAngle(Constants.Coral.CoralServo.DOOR_CLOSED_ANGLE)));
    }

    public Command coralOutputCommand() {
        return rotateToPositionCommand(Constants.Coral.RotationMotor.SCORE_POSITION_ENCODER_VALUE)
                .andThen(runOnce(() -> doorServo.setAngle(Constants.Coral.CoralServo.DOOR_OPEN_ANGLE)))
                //.wait(3000)
                .andThen(() -> rotateToPosition(Constants.Coral.RotationMotor.START_POSITION_ENCODER_VALUE));
    }

    public Command servoRotateToOpen(){
        return runOnce(() -> doorServo.setAngle(Constants.Coral.CoralServo.DOOR_OPEN_ANGLE));
    }

    public Command servoRotateToClosed(){
        return runOnce(() -> doorServo.setAngle(Constants.Coral.CoralServo.DOOR_CLOSED_ANGLE));
    }

    public void rotateToPosition(double encoderValue) {
        coralRotationalPIDController.setReference(encoderValue, SparkMax.ControlType.kPosition);
    }
}
