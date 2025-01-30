package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class LiftSubsystem 
{
    private static SparkMax moduleLiftMotor = new SparkMax(Constants.Lift.LIFT_MOTOR_ID, MotorType.kBrushless);
  
  
  /** Creates a new AmpSubsystem. */
  public LiftSubsystem() {    

  }

  public void startMotor()
  {
    moduleLiftMotor.set(.1);
  }

  public void reverseMotor()
  {
    moduleLiftMotor.set(-0.1);
  }
}
