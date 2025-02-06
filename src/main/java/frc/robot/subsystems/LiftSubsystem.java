package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;

public class LiftSubsystem extends SubsystemBase
{
    private static SparkMax liftMotor = new SparkMax(Constants.Lift.LIFT_MOTOR_ID, MotorType.kBrushless);
    private static SparkClosedLoopController liftPIDController = liftMotor.getClosedLoopController();
    private static SparkMaxConfig liftPIDConfig= new SparkMaxConfig();
  
  /** Creates a new AmpSubsystem. */
  public LiftSubsystem() 
  {    
    liftPIDConfig.closedLoop.pid(Constants.Lift.PID_P, Constants.Lift.PID_I, Constants.Lift.PID_D);
  }

  public Command goToStartStageCommand()
  {
    return runOnce(()->startStage());
  }

  public Command goToCatchStageCommand()
  {
    return runOnce(()->catchStage());
  }

  public Command goToLiftStageCommand()
  {
    return runOnce(()->liftStage());
  }

  public void startStage()
  {
    liftPIDController.setReference(Constants.Lift.START_STAGE_ENCODER_VALUE, SparkMax.ControlType.kPosition);

  }

  public void catchStage()
  {
    liftPIDController.setReference(Constants.Lift.CATCH_STAGE_ENCODER_VALUE, SparkMax.ControlType.kPosition);
  }

  public void liftStage()
  {
    liftPIDController.setReference(Constants.Lift.LIFT_STAGE_ENCODER_VALUE, SparkMax.ControlType.kPosition);

  }
}
