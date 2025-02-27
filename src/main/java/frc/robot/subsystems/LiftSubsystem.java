package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.hardware.TalonFX;


public class LiftSubsystem extends SubsystemBase
{ 
  private static TalonFX liftMotor = new TalonFX(Constants.Lift.LIFT_MOTOR_ID);

public Command runLiftMotorCommand()
{
  return run(()->runLiftMotor());
}

public Command stopLiftMotorCommand()
{
  return run(()->stopLiftMotor());
}

  public void runLiftMotor()
  {
    liftMotor.set(.1);
  }

  public void stopLiftMotor()
  {
    liftMotor.set(0);
  }
}
