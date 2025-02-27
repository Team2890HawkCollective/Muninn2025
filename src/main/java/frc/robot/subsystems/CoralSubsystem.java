package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.playingwithfusion.TimeOfFlight;

public class CoralSubsystem extends SubsystemBase
{
    private static SparkMax coralRotationalMotor = new SparkMax(Constants.Coral.RotationMotor.CORAL_MOTOR_ID, MotorType.kBrushless);
    private static SparkMaxConfig coralRotationalPIDConfig= new SparkMaxConfig();
    private static SparkClosedLoopController coralRotationalPIDController;

//    private static SparkMax coralWheelMotor = new SparkMax(Constants.Coral.WheelMotor.WHEEL_MOTOR_ID, MotorType.kBrushless);

    public TimeOfFlight TOFSensor = new TimeOfFlight(Constants.Coral.WheelMotor.TOF_SENSOR);
    private static Servo doorServo = new Servo(Constants.Coral.Servo.SERVO_PWM_PORT);

    public CoralSubsystem()
    {
        coralRotationalPIDConfig.closedLoop.pid(Constants.Coral.RotationMotor.PID_P, Constants.Coral.RotationMotor.PID_I, Constants.Coral.RotationMotor.PID_D);
        coralRotationalMotor.configure(coralRotationalPIDConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        coralRotationalPIDController = coralRotationalMotor.getClosedLoopController();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler 
        SmartDashboard.putNumber("Coral Absolute Encoder", coralRotationalMotor.getAbsoluteEncoder().getPosition());
        SmartDashboard.putNumber("Coral Relative Encoder", coralRotationalMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Coral Alternate Encoder", coralRotationalMotor.getAlternateEncoder().getPosition());
    }

    public Command rotateToPositionCommand(double encoderValue)
    {
        return runOnce(()->rotateToPosition(encoderValue));
    }

    public Command coralIntakeCommand()
    {
        return rotateToPositionCommand(Constants.Coral.RotationMotor.START_POSITION_ENCODER_VALUE)
        .andThen(runOnce(()->doorServo.setAngle(Constants.Coral.Servo.DOOR_OPEN_ANGLE)))
        //.andThen(runOnce(()->intakeCoral()))
        //.until(() ->{return (TOFSensor.getRange() < Constants.Coral.WheelMotor.TOF_TRIGGER_DIST);})
        //.andThen(()->stopWheels())
        .andThen(runOnce(()->doorServo.setAngle(Constants.Coral.Servo.DOOR_CLOSED_ANGLE)))
        .andThen(()-> rotateToPosition(Constants.Coral.RotationMotor.START_POSITION_ENCODER_VALUE));
    }

    public Command coralOutputCommand(double encoderValue)
    {
        return rotateToPositionCommand(encoderValue)
        .andThen(runOnce(()->doorServo.setAngle(Constants.Coral.Servo.DOOR_OPEN_ANGLE)))
        //.andThen(runOnce(()->outputCoral()))
        //.withTimeout(Constants.Coral.WheelMotor.OUTPUT_DELAY)
        //.andThen(()->stopWheels())
        .andThen(runOnce(()->doorServo.setAngle(Constants.Coral.Servo.DOOR_CLOSED_ANGLE)))
        .andThen(()-> rotateToPosition(Constants.Coral.RotationMotor.START_POSITION_ENCODER_VALUE));
    }

    public void rotateToPosition(double encoderValue)
    {
        coralRotationalPIDController.setReference(encoderValue,SparkMax.ControlType.kPosition);
    }

    public void intakeCoral()
    {
       // coralWheelMotor.set(Constants.Coral.WheelMotor.INTAKE_SPEED);
    }

    public void outputCoral()
    {
        //coralWheelMotor.set(Constants.Coral.WheelMotor.OUTPUT_SPEED);
    }

    public void stopWheels()
    {
        //coralWheelMotor.set(0);
    }
}
