// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.TargetingSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.security.CodeSigner;
import java.util.function.DoubleSupplier;

import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer
{
  private final LiftSubsystem m_LiftSubsystem = new LiftSubsystem();
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  private final CoralSubsystem m_CoralSubsystem = new CoralSubsystem();
  private final AlgaeSubsystem m_AlgaeSubsystem = new AlgaeSubsystem();
  private final TargetingSubsystem m_TargetingSubsystem = new TargetingSubsystem();

  private final CommandJoystick leftButtons = new CommandJoystick(2);
  private final CommandJoystick rightButtons = new CommandJoystick(3);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverXbox = new CommandXboxController(0);
  private final CommandXboxController assistantDriverXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve"));

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(() -> driverXbox.getRightX() * -1,
      driverXbox::getRightY)
      .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative
   * input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
      .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> -driverXbox.getLeftY(),
      () -> -driverXbox.getLeftX())
      .withControllerRotationAxis(() -> driverXbox.getRawAxis(
          2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
      .withControllerHeadingAxis(() -> Math.sin(
          driverXbox.getRawAxis(
              2) *
              Math.PI)
          *
          (Math.PI *
              2),
          () -> Math.cos(
              driverXbox.getRawAxis(
                  2) *
                  Math.PI)
              *
              (Math.PI *
                  2))
      .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    //Autonomous Command Registration
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */

  private void configureBindings()
  {    
    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);
         
    //if(Constants.ShuffleboardConstants.CONTROL_MODE.equalsIgnoreCase("manual")){
    //if(ShuffleboardDisplay.getControlModeChoice().equalsIgnoreCase("manual")){
        // Assistant Driver Manual Control
        assistantDriverXbox.y().whileTrue(m_ElevatorSubsystem.moveElevatorUpCommand()).onFalse(m_ElevatorSubsystem.stopElevatorMotorCommand()); // Manual Elevator Up
        assistantDriverXbox.a().whileTrue(m_ElevatorSubsystem.moveElevatorDownCommand()).onFalse(m_ElevatorSubsystem.stopElevatorMotorCommand()); // Manual Elevator Down
        assistantDriverXbox.b().onTrue(m_ElevatorSubsystem.stopElevatorMotorCommand());
        assistantDriverXbox.povLeft().onTrue(m_AlgaeSubsystem.AlgaeStartCommand()); // Algae Start Position
        assistantDriverXbox.povUp().onTrue(m_AlgaeSubsystem.AlgaeCarryCommand()); // Algae Carry Position
        assistantDriverXbox.povRight().onTrue(m_AlgaeSubsystem.AlgaeOutputCommand()); // Algae Output Position
        assistantDriverXbox.leftBumper().onTrue(m_CoralSubsystem.rotateToPositionCommand(Constants.Coral.RotationMotor.START_POSITION_ENCODER_VALUE));
        assistantDriverXbox.rightBumper().onTrue(m_CoralSubsystem.rotateToPositionCommand(Constants.Coral.RotationMotor.SCORE_POSITION_ENCODER_VALUE));

    //} else {
        // Elevator Stage Buttons
        leftButtons.button(1).onTrue(m_ElevatorSubsystem.goToElevatorStageCommand(6).andThen(new WaitCommand(1)).andThen(m_AlgaeSubsystem.AlgaeOutputCommand())); // Algae L3
        leftButtons.button(2).onTrue(m_ElevatorSubsystem.goToElevatorStageCommand(5).andThen(new WaitCommand(1)).andThen(m_AlgaeSubsystem.AlgaeOutputCommand())); // Algae L2
        leftButtons.button(3).onTrue(m_ElevatorSubsystem.goToElevatorStageCommand(4).andThen(new WaitCommand(1)).andThen(m_CoralSubsystem.coralOutputCommand())); // Coral L4
        leftButtons.button(4).onTrue(m_ElevatorSubsystem.goToElevatorStageCommand(3).andThen(new WaitCommand(1)).andThen(m_CoralSubsystem.coralOutputCommand())); // Coral L3
        leftButtons.button(5).onTrue(m_ElevatorSubsystem.goToElevatorStageCommand(2).andThen(new WaitCommand(1)).andThen(m_CoralSubsystem.coralOutputCommand())); // Coral L2; Skips Coral L1
        leftButtons.button(6).onTrue(m_AlgaeSubsystem.AlgaeCarryCommand().andThen(m_CoralSubsystem.rotateToPositionCommand(Constants.Coral.RotationMotor.START_POSITION_ENCODER_VALUE)).andThen(m_ElevatorSubsystem.goToHomeCommand())); // Elevator All The Way Down
    //}
    // Driver Controls
    driverXbox.leftBumper().onTrue(m_CoralSubsystem.servoRotateToOpen()); // Open Coral Servo
    driverXbox.rightBumper().onTrue(m_CoralSubsystem.servoRotateToClosed()); // Close Coral Servo

    driverXbox.leftTrigger().whileTrue(m_AlgaeSubsystem.moveInputAlgaeWheelsCommand()).onFalse(m_AlgaeSubsystem.stopAlgaeWheelsCommand()); // Intake Algae
    driverXbox.rightTrigger().whileTrue(m_AlgaeSubsystem.moveOutputAlgaeWheelsCommand()).onFalse(m_AlgaeSubsystem.stopAlgaeWheelsCommand()); // Output Algae
    driverXbox.b().onTrue(m_AlgaeSubsystem.stopAlgaeWheelsCommand());

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {

      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());


    }
    if (DriverStation.isTest()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else {
    /*
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.b().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());
    */
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public Command getHomingCommand()
  {
    // An example command will be run in autonomous
    return m_ElevatorSubsystem.goToHomeCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

}
