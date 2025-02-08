package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase
{
    private static SparkMax coralMotor = new SparkMax(Constants.Coral.CORAL_MOTOR_ID, MotorType.kBrushless);
    private static SparkMaxConfig coralPIDConfig= new SparkMaxConfig();
    private static SparkClosedLoopController coralPIDController;

    public AlgaeSubsystem()
    {
        coralPIDConfig.closedLoop.pid(Constants.Coral.PID_P, Constants.Coral.PID_I, Constants.Coral.PID_D);
        coralMotor.configure(coralPIDConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        coralPIDController = coralMotor.getClosedLoopController();
    }

    public Command goToStartPositionCommand()
    {
        return runOnce(()->goToStartPosition());
    }

    public Command goToScorePositionCommand()
    {
        return runOnce(()->goToScorePosition());
    }

    public Command goToCatchPositionCommand()
    {
        return runOnce(()->goToCatchPosition());
    }

    public void goToStartPosition()
    {
        coralPIDController.setReference(Constants.Coral.START_POSITION_ENCODER_VALUE,SparkMax.ControlType.kPosition);
    }

    public void goToScorePosition()
    {
        coralPIDController.setReference(Constants.Coral.SCORE_POSITION_ENCODER_VALUE, SparkMax.ControlType.kPosition);
    }

    public void goToCatchPosition()
    {
        coralPIDController.setReference(Constants.Coral.START_POSITION_ENCODER_VALUE,SparkMax.ControlType.kPosition);
    }
}
