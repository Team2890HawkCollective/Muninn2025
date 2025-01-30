package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.SparkClosedLoopController;


public class ElevatorSubsystem extends SubsystemBase
{
    private static SparkFlex raiseMotor = new SparkFlex(Constants.Elevator.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
    private ClosedLoopConfig elevatorPID = new ClosedLoopConfig();

    public ElevatorSubsystem()
    {
        elevatorPID.pid(Constants.Elevator.PID_P, Constants.Elevator.PID_I, Constants.Elevator.PID_D);  
    }

    public Command moveElevatorCommand()
    {
        return
    }

}
