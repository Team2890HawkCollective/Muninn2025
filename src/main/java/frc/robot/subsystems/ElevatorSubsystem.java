package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.spark.SparkClosedLoopController;

public class ElevatorSubsystem extends SubsystemBase {
    private static SparkFlex elevatorMotor = new SparkFlex(Constants.Elevator.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
    private static SparkClosedLoopController elevatorPIDController;
    public static SparkFlexConfig elevatorPIDConfig = new SparkFlexConfig();
    public DigitalInput bottomlimitSwitch = new DigitalInput(Constants.Elevator.LIMIT_SWITCH_PWM_PORT);

    public ElevatorSubsystem() {
        elevatorPIDConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control. We don't need to pass a closed loop
                // slot, as it will default to slot 0.
                .p(Constants.Elevator.PID_P)
                .i(Constants.Elevator.PID_I)
                .d(Constants.Elevator.PID_D)
                .outputRange(-1, 1)
                // Set PID values for velocity control in slot 1
                .p(0.0001, ClosedLoopSlot.kSlot1)
                .i(0, ClosedLoopSlot.kSlot1)
                .d(0, ClosedLoopSlot.kSlot1)
                .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot1);
        elevatorMotor.configure(elevatorPIDConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        elevatorPIDController = elevatorMotor.getClosedLoopController();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler
        SmartDashboard.putNumber("Elevator Relative Encoder", elevatorMotor.getEncoder().getPosition());
        SmartDashboard.putBoolean("Elevator Limit Switch State", bottomlimitSwitch.get());
    }

    public Command goToElevatorStageCommand(int elevatorStageValue) {
        if (elevatorStageValue == 0) {
            return runOnce(() -> moveElevatorDownCommand());
        } else {
            return runOnce(() -> goToElevatorStage(elevatorStageValue));
        }
    }

    public void goToElevatorStage(int elevatorStageValue) {
        elevatorPIDController.setReference(
                Constants.Elevator.STAGE_ENCODER_DIFFERENCES[elevatorStageValue]
                        + Constants.Elevator.BASE_STAGE_ENCODER_VALUE,
                SparkFlex.ControlType.kPosition);
    }

    public Command goToHomeCommand() {
        return run(() -> moveElevatorDown()).until(() -> bottomlimitSwitch.get() == true)
                .andThen(() -> stopElevatorMotor())
                .andThen(() -> Constants.Elevator.BASE_STAGE_ENCODER_VALUE = elevatorMotor.getEncoder().getPosition());
    }

    public Command moveElevatorUpCommand() {
        return run(() -> moveElevatorUp());
    }

    public Command moveElevatorDownCommand() {
        return run(() -> moveElevatorDown()).until(() -> bottomlimitSwitch.get() == true)
                .andThen(() -> stopElevatorMotor())
                .andThen(() -> zeroEncoder());
    }

    public Command joystickMoveElevatorCommand(double joystickY) {
        return run(() -> joysticMoveElevatorUp(joystickY))
                .onlyWhile(() -> (Math.abs(joystickY) > Constants.Elevator.DEADZONE || !bottomlimitSwitch.get()))
                .andThen(() -> stopElevatorMotorCommand());
    }

    public void moveElevatorUp() {
        elevatorMotor.set(Constants.Elevator.ELEVATOR_UP_SPEED);
    }

    public void joysticMoveElevatorUp(double speed) {
        elevatorMotor.set(speed);
    }

    public void moveElevatorDown() {
        elevatorMotor.set(Constants.Elevator.HOMING_SPEED);
    }

    public void stopElevatorMotor() {
        elevatorMotor.set(0);
    }

    public Command stopElevatorMotorCommand() {
        return runOnce(() -> stopElevatorMotor());
    }

    public void zeroEncoder() {
        if (bottomlimitSwitch.get() == true)
            Constants.Elevator.BASE_STAGE_ENCODER_VALUE = elevatorMotor.getEncoder().getPosition();
    }

}
