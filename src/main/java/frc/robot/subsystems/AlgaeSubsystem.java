package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase {
    private static SparkMax algaeRotationMotor = new SparkMax(Constants.Algae.Rotation.ALGAE_ROTATION_MOTOR_ID,
            MotorType.kBrushless);
    private static SparkMaxConfig algaeRotationPIDConfig = new SparkMaxConfig();
    private static SparkClosedLoopController algaeRotationPIDController;

    // private static SparkMax algaeWheelMotor = new
    // SparkMax(Constants.Algae.Wheel.ALGAE_WHEEL_MOTOR_ID, MotorType.kBrushless);

    public AlgaeSubsystem() {
        algaeRotationPIDConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control. We don't need to pass a closed loop
                // slot, as it will default to slot 0.
                .p(Constants.Algae.Rotation.PID_P)
                .i(Constants.Algae.Rotation.PID_I)
                .d(Constants.Algae.Rotation.PID_D)
                .outputRange(-1, 1)
                // Set PID values for velocity control in slot 1
                .p(0.0001, ClosedLoopSlot.kSlot1)
                .i(0, ClosedLoopSlot.kSlot1)
                .d(0, ClosedLoopSlot.kSlot1)
                .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot1);
        algaeRotationMotor.configure(algaeRotationPIDConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
        algaeRotationPIDController = algaeRotationMotor.getClosedLoopController();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler
        SmartDashboard.putNumber("Algae Absolute Encoder", algaeRotationMotor.getAbsoluteEncoder().getPosition());
        SmartDashboard.putNumber("Algae Relative Encoder", algaeRotationMotor.getEncoder().getPosition());
    }

    public Command rotateToPositionCommand(double encoderValue) {
        return runOnce(() -> rotateToStartPosition(encoderValue));
    }

    public Command moveAlgaeWheelsCommand(double speed) {
        return runOnce(() -> moveAlgaeWheels(speed));
    }

    public void rotateTest() {
        algaeRotationMotor.set(.7);
    }

    public Command rotateTestCommand() {
        return run(() -> rotateTest());
    }

    public void rotateToStartPosition(double encoderValue) {
        algaeRotationPIDController.setReference(encoderValue, SparkMax.ControlType.kPosition);
    }

    public void stopMotor() {
        algaeRotationMotor.set(0);
    }

    public Command stopMotorCommand() {
        return runOnce(() -> stopMotor());
    }

    public void moveAlgaeWheels(double speed) {
        // algaeWheelMotor.set(speed);
    }

}
