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

public class CoralSubsystem extends SubsystemBase
{
    private static SparkMax coralRotationalMotor = new SparkMax(Constants.Coral.RotationMotor.CORAL_MOTOR_ID, MotorType.kBrushless);
    private static SparkMaxConfig coralRotationalPIDConfig= new SparkMaxConfig();
    private static SparkClosedLoopController coralRotationalPIDController;

    public CoralSubsystem()
    {
        coralRotationalPIDConfig.closedLoop.pid(Constants.Coral.RotationMotor.PID_P, Constants.Coral.RotationMotor.PID_I, Constants.Coral.RotationMotor.PID_D);
        coralRotationalMotor.configure(coralRotationalPIDConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        coralRotationalPIDController = coralRotationalMotor.getClosedLoopController();
    }

    public Command rotateToStartPositionCommand()
    {
        return runOnce(()->rotateToStartPosition());
    }

    public Command rotateToScorePositionCommand()
    {
        return runOnce(()->rotateToScorePosition());
    }

    public Command rotateToCatchPositionCommand()
    {
        return runOnce(()->rotateToCatchPosition());
    }

    public void rotateToStartPosition()
    {
        coralRotationalPIDController.setReference(Constants.Coral.RotationMotor.START_POSITION_ENCODER_VALUE,SparkMax.ControlType.kPosition);
    }

    public void rotateToScorePosition()
    {
        coralRotationalPIDController.setReference(Constants.Coral.RotationMotor.SCORE_POSITION_ENCODER_VALUE, SparkMax.ControlType.kPosition);
    }

    public void rotateToCatchPosition()
    {
        coralRotationalPIDController.setReference(Constants.Coral.RotationMotor.START_POSITION_ENCODER_VALUE,SparkMax.ControlType.kPosition);
    }
}
