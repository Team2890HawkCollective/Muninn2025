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

    // TODO: Add limit switch
    DigitalInput bottomlimitSwitch = new DigitalInput(Constants.Elevator.LIMIT_SWITCH_PWM_PORT);

    // TODO: Add potentiometer
    // AnalogPotentiometer potentiometer = new
    // AnalogPotentiometer(Constants.Elevator.Potentiometer.PWM_PORT,
    // Constants.Elevator.Potentiometer.UPPER_BOUND,
    // Constants.Elevator.Potentiometer.LOWER_BOUND);

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
        SmartDashboard.putNumber("Elevator Absolute Encoder", elevatorMotor.getAbsoluteEncoder().getPosition());
        SmartDashboard.putNumber("Elevator Relative Encoder", elevatorMotor.getEncoder().getPosition());
        SmartDashboard.putBoolean("Limit Switch State", bottomlimitSwitch.get());
    }

    public Command goToStageEncoderCommand(double encoderValue) {
        return runOnce(() -> goToStageEncoder(encoderValue));
    }

    public void goToStageEncoder(double encoderValue) {
        elevatorPIDController.setReference(encoderValue, SparkFlex.ControlType.kPosition);
    }

    /*
     * public Command goToStagePotentiometerCommand(double potentiometerValue)
     * {
     * return runOnce(() -> goToStagePotentiometer(potentiometerValue));
     * }
     * 
     * public void goToStagePotentiometer(double potentiometerValue)
     * {
     * double difference = potentiometer.get() - potentiometerValue;
     * while (Math.abs(difference) >
     * Constants.Elevator.Potentiometer.MOVEMENT_TOLERATION) {
     * if (difference > 0)
     * elevatorMotor.set(Constants.Elevator.POTENTIOMETER_MOVEMENT_SPEED);
     * else
     * elevatorMotor.set(Constants.Elevator.POTENTIOMETER_MOVEMENT_SPEED * -1);
     * 
     * difference = potentiometer.get() - potentiometerValue;
     * }
     * elevatorMotor.set(0);
     * 
     * }
     */
    public Command goToHomeCommand() {
        return runOnce(() -> homingSequence());
    }

    public Command moveElevatorUpCommand() {
        return run(() -> moveElevatorUp());
    }

    public Command moveElevatorDownCommand() {
        return run(() -> moveElevatorDown());
        // .until(()-> {return bottomlimitSwitch.get() == true;})
        // .andThen(stopElevatorMotorCommand());

    }

    public void homingSequence() {
        while (bottomlimitSwitch.get() == false) {
            elevatorMotor.set(Constants.Elevator.HOMING_SPEED);
        }
        elevatorMotor.set(0);
        elevatorMotor.getEncoder().setPosition(0);
    }

    public void moveElevatorUp() {
        elevatorMotor.set(-0.3);
    }

    public void moveElevatorDown() {
        elevatorMotor.set(0.3);
    }

    public void stopElevatorMotor() {
        elevatorMotor.set(0);
    }

    public Command stopElevatorMotorCommand() {
        return runOnce(() -> stopElevatorMotor());
    }

}
