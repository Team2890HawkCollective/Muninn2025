package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
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
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Elevator;

import com.ctre.phoenix6.controls.Follower;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;//SparkClosedLoopController;
import com.revrobotics.spark.config.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class ElevatorSubsystem extends SubsystemBase {
    private static SparkFlex elevatorMotor1 = new SparkFlex(Constants.Elevator.ELEVATOR_MOTOR1_ID,
            MotorType.kBrushless);
    private static RelativeEncoder elevator1Encoder;
    private static SparkClosedLoopController elevator1PIDController;
    public static SparkFlexConfig elevator1PIDConfig = new SparkFlexConfig();

    private static SparkFlex elevatorMotor2;
    public static SparkFlexConfig elevator2PIDConfig = new SparkFlexConfig();
    // private static SparkClosedLoopController elevator2PIDController;

    // private static SparkFlex elevatorMotor2 = new
    // SparkFlex(Constants.Elevator.ELEVATOR_MOTOR2_ID, MotorType.kBrushless);
    public DigitalInput bottomlimitSwitch = new DigitalInput(Constants.Elevator.LIMIT_SWITCH_PWM_PORT);
    // elevatorMotor2.SparkFlexConfig.Follower
    // elevatorMotor2.SparkF;
    // public elevatorMotor2


    private final ElevatorFeedforward elevatorFeedforward;
    private final TrapezoidProfile elevatorTrapezoidProfile;
    private TrapezoidProfile.State elevatorGoal = new TrapezoidProfile.State();
    private TrapezoidProfile.State elevatorSetpoint;

    private double relativeEncoderHeightMeters =  0;
    private double lastGoalPositionMeters = Constants.Elevator.TrapezoidProfile.kElevatorMinHeightMeters;

    public ElevatorSubsystem() {

        elevator1PIDController = elevatorMotor1.getClosedLoopController();

        elevatorFeedforward = new ElevatorFeedforward(Constants.Elevator.TrapezoidProfile.kS, Constants.Elevator.TrapezoidProfile.kG, Constants.Elevator.TrapezoidProfile.kV);

	    elevatorTrapezoidProfile = new TrapezoidProfile(new Constraints(elevatorFeedforward.maxAchievableVelocity(12.0, Constants.Elevator.TrapezoidProfile.kElevatorMaxAccelerationMPSPS),
        Constants.Elevator.TrapezoidProfile.kElevatorMaxAccelerationMPSPS));


        //elevatorTrapezoidProfile = new TrapezoidProfile(new Constraints(Constants.Elevator.TrapezoidProfile.kElevatorMaxAccelerationMPSPS, Constants.Elevator.TrapezoidProfile.kElevatorMaxAccelerationMPSPS)); 

        // Creates a SysIdRoutine
        //SysIdRoutine routine = new SysIdRoutine(
        //    new SysIdRoutine.Config(),
        //    new SysIdRoutine.Mechanism(this::voltageDrive, this::logMotors, this)
        //);

        elevator1PIDConfig.closedLoop
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
        elevator1PIDConfig.smartCurrentLimit(80);
        elevatorMotor1.configure(elevator1PIDConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevator1Encoder = elevatorMotor1.getEncoder();

        elevator1PIDConfig.encoder
            .positionConversionFactor(Constants.Elevator.TrapezoidProfile.kElevatorEncoderPositionConversionFactor)
            .velocityConversionFactor(Constants.Elevator.TrapezoidProfile.kElevatorEncoderVelocityConversionFactor);

        elevatorMotor2 = new SparkFlex(Constants.Elevator.ELEVATOR_MOTOR2_ID, MotorType.kBrushless);
        // elevator2PIDController = elevatorMotor1.getClosedLoopController();
        elevatorMotor2.configure(
                elevator2PIDConfig.follow(elevatorMotor1),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        elevator1Encoder.setPosition(Constants.Elevator.TrapezoidProfile.elevatorHomeHeightMeters); 

        elevatorSetpoint = new TrapezoidProfile.State(elevator1Encoder.getPosition(), elevator1Encoder.getVelocity());

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler
        SmartDashboard.putNumber("Elevator Relative Encoder", elevatorMotor1.getEncoder().getPosition());
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
        elevator1PIDController.setReference(Constants.Elevator.STAGE_ENCODER_DIFFERENCES[elevatorStageValue] + Constants.Elevator.BASE_STAGE_ENCODER_VALUE, SparkFlex.ControlType.kPosition,ClosedLoopSlot.kSlot0);
    }

    public Command goToHomeCommand() {
        return run(() -> moveElevatorDown()).until(() -> bottomlimitSwitch.get() == true)
                .andThen(() -> stopElevatorMotor())
                .andThen(() -> Constants.Elevator.BASE_STAGE_ENCODER_VALUE = elevatorMotor1.getEncoder().getPosition())
                .andThen(() -> zeroEncoder());
    }

    public Command holdPositionCommand() {
        double position = elevatorMotor1.getEncoder().getPosition();
        return runOnce(() -> elevator1PIDController.setReference(position,SparkFlex.ControlType.kPosition));
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
        elevatorMotor1.set(Constants.Elevator.ELEVATOR_UP_SPEED);
        // elevatorMotor2.set(Constants.Elevator.ELEVATOR_UP_SPEED);
    }

    public void joysticMoveElevatorUp(double speed) {
        elevatorMotor1.set(speed);
        // elevatorMotor2.set(speed);
    }

    public void moveElevatorDown() {
        elevatorMotor1.set(Constants.Elevator.HOMING_SPEED);
        // elevatorMotor2.set(Constants.Elevator.HOMING_SPEED);
    }

    public void stopElevatorMotor() {
        elevatorMotor1.set(0);
        // elevatorMotor2.set(0);
    }

    public Command stopElevatorMotorCommand() {
        return runOnce(() -> stopElevatorMotor());
    }

    public void zeroEncoder() {
        if (bottomlimitSwitch.get() == true)
            Constants.Elevator.BASE_STAGE_ENCODER_VALUE = elevatorMotor1.getEncoder().getPosition();
    }
     
    
    public void setGoalPositionMeters(double goalPositionMeters) {
        if (goalPositionMeters < Constants.Elevator.TrapezoidProfile.kElevatorMinHeightMeters) {
          goalPositionMeters = Constants.Elevator.TrapezoidProfile.kElevatorMinHeightMeters;
        } else if (goalPositionMeters > Constants.Elevator.TrapezoidProfile.kElevatorMaxHeightMeters) {
          goalPositionMeters = Constants.Elevator.TrapezoidProfile.kElevatorMaxHeightMeters;
        }
    
        lastGoalPositionMeters = goalPositionMeters;
        
        elevatorGoal.position = goalPositionMeters;
        elevatorGoal.velocity = 0.0;
    
        elevatorSetpoint.position = elevator1Encoder.getPosition();
        elevatorSetpoint.velocity = 0.0;
        }
    
      public double getHeightMeters(){
        return relativeEncoderHeightMeters;
      }
    

    public void runClosedLoop() {
        elevatorSetpoint = elevatorTrapezoidProfile.calculate((Constants.Elevator.kDt), elevatorSetpoint, elevatorGoal);
        double arbFF = elevatorFeedforward.calculate(elevatorSetpoint.velocity);
        elevator1PIDController.setReference(elevatorSetpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, arbFF);
        SmartDashboard.putNumber("Elevator FeedForward", arbFF);
        
    }
  
}