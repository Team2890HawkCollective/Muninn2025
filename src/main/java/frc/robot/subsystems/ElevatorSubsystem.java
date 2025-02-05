package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.spark.SparkClosedLoopController;


public class ElevatorSubsystem extends SubsystemBase
{
    private static SparkFlex raiseMotor = new SparkFlex(Constants.Elevator.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
    private static SparkClosedLoopController elevatorPIDController = raiseMotor.getClosedLoopController();
    public static SparkFlexConfig elevatorPIDConfig = new SparkFlexConfig();

    public ElevatorSubsystem()
    {        
        elevatorPIDConfig.closedLoop.pid(Constants.Elevator.PID_P, Constants.Elevator.PID_I, Constants.Elevator.PID_D);
    }

    public Command goToBaseStageCommand()
    {
        return runOnce(()->goToBaseStage());
    }

    public Command goToFirstStageCommand()
    {
        return runOnce(()->goToFirstStage());
    }

    public Command goToSecondStageCommand()
    {
        return runOnce(()->goToSecondStage());
    }

    public Command goToThirdStageCommand()
    {
        return runOnce(()->goToThirdStage());
    }

    public Command goToFourthStageCommand()
    {
        return runOnce(()->goToFourthStage());
    }

    public void goToBaseStage() 
    {
        elevatorPIDController.setReference(Constants.Elevator.BASE_STAGE_ENCODER_VALUE, SparkFlex.ControlType.kPosition);
    }

    public void goToFirstStage() 
    {
        elevatorPIDController.setReference(Constants.Elevator.FIRST_CORAL_STAGE_ENCODER_VALUE, SparkFlex.ControlType.kPosition);
    }

    public void goToSecondStage() 
    {
        elevatorPIDController.setReference(Constants.Elevator.SECOND_CORAL_STAGE_ENCODER_VALUE, SparkFlex.ControlType.kPosition);
    }

    public void goToThirdStage() 
    {
        elevatorPIDController.setReference(Constants.Elevator.THIRD_CORAL_STAGE_ENCODER_VALUE, SparkFlex.ControlType.kPosition);
    }

    public void goToFourthStage() 
    {
        elevatorPIDController.setReference(Constants.Elevator.FOURTH_CORAL_STAGE_ENCODER_VALUE, SparkFlex.ControlType.kPosition);
    }
/* 

    public Command moveElevatorCommand()
    {
        return
    }
*/
}
