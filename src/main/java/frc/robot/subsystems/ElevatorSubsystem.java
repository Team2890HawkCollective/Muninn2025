package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.spark.SparkClosedLoopController;

public class ElevatorSubsystem extends SubsystemBase 
{
    private static SparkFlex elevatorMotor = new SparkFlex(Constants.Elevator.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
    private static SparkClosedLoopController elevatorPIDController;
    public static SparkFlexConfig elevatorPIDConfig = new SparkFlexConfig();

    // TODO: Add limit switch
    DigitalInput bottomlimitSwitch = new DigitalInput(Constants.Elevator.LIMIT_SWITCH_PWM_PORT);


    // TODO: Add potentiometer
    //AnalogPotentiometer potentiometer = new AnalogPotentiometer(Constants.Elevator.Potentiometer.PWM_PORT,
            //Constants.Elevator.Potentiometer.UPPER_BOUND, Constants.Elevator.Potentiometer.LOWER_BOUND);

    public ElevatorSubsystem() 
    {
        elevatorPIDConfig.closedLoop.pid(Constants.Elevator.PID_P, Constants.Elevator.PID_I, Constants.Elevator.PID_D);
        elevatorMotor.configure(elevatorPIDConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        elevatorPIDController = elevatorMotor.getClosedLoopController();
    }

    public Command goToStageEncoderCommand(double encoderValue) 
    {
        return runOnce(() -> goToStageEncoder(encoderValue));
    }

    public void goToStageEncoder(double encoderValue) 
    {
        elevatorPIDController.setReference(encoderValue, SparkFlex.ControlType.kPosition);
    }

    /*
    public Command goToStagePotentiometerCommand(double potentiometerValue) 
    {
        return runOnce(() -> goToStagePotentiometer(potentiometerValue));
    }

    public void goToStagePotentiometer(double potentiometerValue) 
    {
        double difference = potentiometer.get() - potentiometerValue;
        while (Math.abs(difference) > Constants.Elevator.Potentiometer.MOVEMENT_TOLERATION) {
            if (difference > 0)
                elevatorMotor.set(Constants.Elevator.POTENTIOMETER_MOVEMENT_SPEED);
            else
                elevatorMotor.set(Constants.Elevator.POTENTIOMETER_MOVEMENT_SPEED * -1);

            difference = potentiometer.get() - potentiometerValue;
        }
        elevatorMotor.set(0);

    }
*/
    public Command goToHomeCommand()
    {
        return runOnce(()->homingSequence());
    }

    public Command moveElevatorUpCommand()
    {
        return run(() -> moveElevatorUp());
    }

    public Command moveElevatorDownCommand()
    {
        return run(() -> moveElevatorDown())
        .until(()-> {return bottomlimitSwitch.get() == true;})
        .andThen(stopElevatorMotorCommand());

    }

    public void homingSequence()
    {
       while(bottomlimitSwitch.get() == false) 
       {
        elevatorMotor.set(Constants.Elevator.HOMING_SPEED);
       }
       elevatorMotor.set(0);
       elevatorMotor.getEncoder().setPosition(0);
    }


    public void moveElevatorUp()
    {
        elevatorMotor.set(0.1);
    }

    public void moveElevatorDown()
    {
        elevatorMotor.set(-0.1);
    }

    public void stopElevatorMotor()
    {
        elevatorMotor.set(0);
    }

    public Command stopElevatorMotorCommand()
    {
        return runOnce(()-> stopElevatorMotor());
    }

     
}
