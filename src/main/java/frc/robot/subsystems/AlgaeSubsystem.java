package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.spark.SparkMax;
import com.playingwithfusion.TimeOfFlight;
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

    private static SparkMax algaeWheelMotor = new SparkMax(Constants.Algae.Wheel.ALGAE_WHEEL_MOTOR_ID,
            MotorType.kBrushless);

    //public DigitalInput algaeLimitSwitch = new DigitalInput(Constants.Algae.Wheel.LIMIT_SWITCH_PORT);
    //public TimeOfFlight TOFSensor = new TimeOfFlight(Constants.Algae.Wheel.TOF_SENSOR);


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
        algaeWheelMotor.getOutputCurrent();
        SmartDashboard.putNumber("Algae Relative Encoder", algaeRotationMotor.getEncoder().getPosition());
        //SmartDashboard.putNumber("Algae TOF Distance", TOFSensor.getRange());
    }

    public Command rotateToPositionCommand(double encoderValue) {
        return runOnce(() -> rotateToPosition(encoderValue));
    }

    
    public void rotateToPosition(double encoderValue) {
        algaeRotationPIDController.setReference(encoderValue, SparkMax.ControlType.kPosition);
    }

    public Command AlgaeCarryCommand() {
        return rotateToPositionCommand(Constants.Algae.Rotation.CARRY_ENCODER_VALUE);

    }
    public Command AlgaeOutputCommand() {
        return rotateToPositionCommand(Constants.Algae.Rotation.COLLECT_ENCODER_VALUE_POS);
    }
    
    public Command AlgaeStartCommand(){
        return rotateToPositionCommand(Constants.Algae.Rotation.START_POSITION_ENCODER_VALUE);
    }

    public Command moveInputAlgaeWheelsCommand(){
        return runOnce(() -> moveInputAlgaeWheels());
    }

    public Command moveOutputAlgaeWheelsCommand(){
        return runOnce(() -> moveOutputAlgaeWheels());
    }

    public Command stopAlgaeWheelsCommand(){
        return runOnce(() -> stopWheels());
    }


    public void stopRotationMotor() {
        algaeRotationMotor.set(0);
    }

    public Command stopRotationMotorCommand() {
        return runOnce(() -> stopRotationMotor());
    }

    public void moveInputAlgaeWheels() {
        //if (algaeLimitSwitch.getRange() > Constants.Algae.Wheel.TOF_DISTANCE) {
        algaeWheelMotor.set(Constants.Algae.Wheel.WHEEL_INTAKE_SPEED);
        //} else {
            //algaeWheelMotor.set(0);
        //}
    }

    public void moveOutputAlgaeWheels() {
        algaeWheelMotor.set(Constants.Algae.Wheel.WHEEL_OUTPUT_SPEED);
    }

    public void stopWheels() {
        algaeWheelMotor.set(0);
    }

    public Command joystickRotateAlgaeCommand(double joystickY) {
        return run(() -> joystickRotateAlgae(joystickY))
            .onlyWhile(() -> (Math.abs(joystickY)>Constants.Algae.Rotation.DEADZONE))
            .andThen(() -> stopRotationMotor());
    }

    public void joystickRotateAlgae(double speed){
        algaeWheelMotor.set(speed);
    }

}
