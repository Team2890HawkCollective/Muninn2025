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
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
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
public class RobotContainer {
  private final LiftSubsystem m_LiftSubsystem = new LiftSubsystem();
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  private final CoralSubsystem m_CoralSubsystem = new CoralSubsystem();
  private final AlgaeSubsystem m_AlgaeSubsystem = new AlgaeSubsystem();

  private final CommandXboxController driverXbox = new CommandXboxController(0);
  private final CommandXboxController driverAssistantXbox = new CommandXboxController(1);
  private final CommandJoystick leftButtons = new CommandJoystick(2);
  // private final CommandJoystick joystick = new CommandJoystick(2);
  private final CommandJoystick rightButtons = new CommandJoystick(3);
  // Replace with CommandPS4Controller or CommandJoystick if needed

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve"));

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(driverXbox::getRightX)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
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

    // Autonomous Command Registration
    NamedCommands.registerCommand("To_Base_Encoder", scoreCoralCommand(Constants.Elevator.BASE_STAGE_ENCODER_VALUE,
        Constants.Coral.RotationMotor.START_POSITION_ENCODER_VALUE));
    // NamedCommands.registerCommand("To_Base_Pot",scoreCoralCommand(Constants.Elevator.Potentiometer.BASE_STAGE_POT_VALUE,
    // Constants.Coral.RotationMotor.START_POSITION_ENCODER_VALUE));
    NamedCommands.registerCommand("Score_Coral_Fourth_Encoder",
        scoreCoralCommand(Constants.Elevator.FOURTH_CORAL_STAGE_ENCODER_VALUE,
            Constants.Coral.RotationMotor.SCORE_POSITION_ENCODER_VALUE));
    // NamedCommands.registerCommand("Score_Coral_Fourth_Pot",scoreCoralCommand(Constants.Elevator.Potentiometer.FOURTH_CORAL_STAGE_POT_VALUE,
    // Constants.Coral.RotationMotor.SCORE_TOP_POSITION_ENCODER_VALUE));
    NamedCommands.registerCommand("Score_Coral_Third_Encoder",
        scoreCoralCommand(Constants.Elevator.THIRD_CORAL_STAGE_ENCODER_VALUE,
            Constants.Coral.RotationMotor.SCORE_POSITION_ENCODER_VALUE));
    // NamedCommands.registerCommand("Score_Coral_Third_Pot",scoreCoralCommand(Constants.Elevator.Potentiometer.THIRD_CORAL_STAGE_POT_VALUE,
    // Constants.Coral.RotationMotor.SCORE_LOWER_POSITION_ENCODER_VALUE));
    NamedCommands.registerCommand("Score_Coral_Second_Encoder",
        scoreCoralCommand(Constants.Elevator.SECOND_CORAL_STAGE_ENCODER_VALUE,
            Constants.Coral.RotationMotor.SCORE_POSITION_ENCODER_VALUE));
    // NamedCommands.registerCommand("Score_Second_Pot",scoreCoralCommand(Constants.Elevator.Potentiometer.SECOND_CORAL_STAGE_POT_VALUE,
    // Constants.Coral.RotationMotor.SCORE_LOWER_POSITION_ENCODER_VALUE));
    NamedCommands.registerCommand("Score_Coral_First_Encoder",
        scoreCoralCommand(Constants.Elevator.FIRST_CORAL_STAGE_ENCODER_VALUE,
            Constants.Coral.RotationMotor.SCORE_POSITION_ENCODER_VALUE));
    // NamedCommands.registerCommand("Score_Coral_First_Pot",scoreCoralCommand(Constants.Elevator.Potentiometer.FIRST_CORAL_STAGE_POT_VALUE,
    // Constants.Coral.RotationMotor.SCORE_LOWER_POSITION_ENCODER_VALUE));
    NamedCommands.registerCommand("Collect_Coral", collectCoralCommand());
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

  private void configureBindings() {
    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    // Scoring Buttons
    // if(joystick.getX() == Constants.OperatorConstants.JOYSTICK_IS_RIGHT)
    // {
    /* */ leftButtons.button(6).onTrue(scoreCoralCommand(Constants.Elevator.BASE_STAGE_ENCODER_VALUE,
        Constants.Coral.RotationMotor.START_POSITION_ENCODER_VALUE));
    // leftButtons.button(6).onTrue(scoreCoralCommand(Constants.Elevator.Potentiometer.BASE_STAGE_POT_VALUE,
    // Constants.Coral.RotationMotor.START_POSITION_ENCODER_VALUE));
    leftButtons.button(2).onTrue(scoreCoralCommand(Constants.Elevator.L4_CORAL_STAGE_ENCODER_VALUE,
        Constants.Coral.RotationMotor.SCORE_POSITION_ENCODER_VALUE));
    // leftButtons.button(2).onTrue(scoreCoralCommand(Constants.Elevator.Potentiometer.FOURTH_CORAL_STAGE_POT_VALUE,
    // Constants.Coral.RotationMotor.SCORE_TOP_POSITION_ENCODER_VALUE));
    leftButtons.button(3).onTrue(scoreCoralCommand(Constants.Elevator.THIRD_CORAL_STAGE_ENCODER_VALUE,
        Constants.Coral.RotationMotor.SCORE_POSITION_ENCODER_VALUE));
    // leftButtons.button(3).onTrue(scoreCoralCommand(Constants.Elevator.Potentiometer.THIRD_CORAL_STAGE_POT_VALUE,
    // Constants.Coral.RotationMotor.SCORE_LOWER_POSITION_ENCODER_VALUE));
    leftButtons.button(4).onTrue(scoreCoralCommand(Constants.Elevator.SECOND_CORAL_STAGE_ENCODER_VALUE,
        Constants.Coral.RotationMotor.SCORE_POSITION_ENCODER_VALUE));
    // leftButtons.button(4).onTrue(scoreCoralCommand(Constants.Elevator.Potentiometer.SECOND_CORAL_STAGE_POT_VALUE,
    // Constants.Coral.RotationMotor.SCORE_LOWER_POSITION_ENCODER_VALUE));
    leftButtons.button(5).onTrue(scoreCoralCommand(Constants.Elevator.FIRST_CORAL_STAGE_ENCODER_VALUE,
        Constants.Coral.RotationMotor.SCORE_POSITION_ENCODER_VALUE));
    // leftButtons.button(5).onTrue(scoreCoralCommand(Constants.Elevator.Potentiometer.FIRST_CORAL_STAGE_POT_VALUE,
    // Constants.Coral.RotationMotor.SCORE_LOWER_POSITION_ENCODER_VALUE));
    leftButtons.button(1).onTrue(collectCoralCommand());
    // }

    // if(joystick.getX() == Constants.OperatorConstants.JOYSTICK_IS_LEFT)
    // {
    // TODO: add algae scoring button commands
    // }
    rightButtons.button(3).onTrue(getHomingCommand());

    driverXbox.rightBumper().onTrue(m_CoralSubsystem.servoRotateToOpen());
    driverXbox.leftBumper().onTrue(m_CoralSubsystem.servoRotateToClosed());
    
    driverXbox.leftTrigger().whileTrue(m_AlgaeSubsystem.moveInputAlgaeWheelsCommand());
    driverXbox.rightTrigger().whileTrue(m_AlgaeSubsystem.moveOutputAlgaeWheelsCommand());
    driverXbox.x().onTrue(m_AlgaeSubsystem.stopAlgaeWheelsCommand());
    driverXbox.y().onTrue(m_AlgaeSubsystem.AlgaeCarryCommand());
    driverXbox.a().onTrue(m_AlgaeSubsystem.AlgaeOutputCommand());
    driverXbox.b().onTrue(m_AlgaeSubsystem.AlgaeStartCommand());
    driverXbox.povDown().onTrue(m_LiftSubsystem.lockRatchetCommand());
    driverXbox.povUp().onTrue(m_LiftSubsystem.retractRatchetCommand());
    
   // driverXbox.a().onTrue(m_LiftSubsystem.);

    driverAssistantXbox.y().whileTrue(m_ElevatorSubsystem.moveElevatorUpCommand());
    driverAssistantXbox.a().whileTrue(m_ElevatorSubsystem.moveElevatorDownCommand());
    driverAssistantXbox.b().onTrue(m_ElevatorSubsystem.stopElevatorMotorCommand());
    driverAssistantXbox.povDown()
        .onTrue(m_CoralSubsystem.rotateToPositionCommand(Constants.Coral.RotationMotor.START_POSITION_ENCODER_VALUE));
    // driverAssistantXbox.povRight().onTrue(m_CoralSubsystem.rotateToPositionCommand(Constants.Coral.RotationMotor.SCORE_POSITION_ENCODER_VALUE));
    driverAssistantXbox.povUp()
        .onTrue(m_CoralSubsystem.rotateToPositionCommand(Constants.Coral.RotationMotor.SCORE_POSITION_ENCODER_VALUE));
    //driverAssistantXbox.leftBumper().onTrue(m_AlgaeSubsystem.joystickRotateAlgaeCommand());
    driverAssistantXbox.leftTrigger().onTrue(m_LiftSubsystem.runLiftMotorCommand());
    driverAssistantXbox.rightTrigger().onTrue(m_LiftSubsystem.stopLiftMotorCommand());
    driverAssistantXbox.x().onTrue(m_CoralSubsystem.coralOutputCommand());
    EventLoop elevatorJoystickLoop = new EventLoop();
    elevatorJoystickLoop.bind(()->m_ElevatorSubsystem.joystickMoveElevatorCommand(driverAssistantXbox.getRightY()).until(m_ElevatorSubsystem.bottomlimitSwitch::get).handleInterrupt(()->m_ElevatorSubsystem.stopElevatorMotorCommand()));
    driverAssistantXbox.rightStick(elevatorJoystickLoop);
    EventLoop algaeJoystickLoop = new EventLoop();
    algaeJoystickLoop.bind(()->m_AlgaeSubsystem.joystickRotateAlgaeCommand(driverAssistantXbox.getLeftY()));
    driverAssistantXbox.leftStick(algaeJoystickLoop);
    // driverAssistantXbox.rightBumper().onTrue(m_AlgaeSubsystem.rotateToPositionCommand(Constants.Algae.Rotation.COLLECT_ENCODER_VALUE_POS));

    /*
     * leftButtons.button(1).onTrue(m_ElevatorSubsystem.moveElevatorUpCommand());
     * leftButtons.button(2).onTrue(m_ElevatorSubsystem.moveElevatorDownCommand());
     * leftButtons.button(3).onTrue(m_ElevatorSubsystem.stopElevatorMotorCommand());
     * 
     * if(joystick.getY() == Constants.OperatorConstants.JOYSTICK_IS_UP)
     * {
     * moveElevatorUpCommand()
     * }
     * 
     * if(joystick.getY() == Constants.OperatorConstants.JOYSTICK_IS_DOWN)
     * {
     * moveElevatorDownCommand()
     * }
     * 
     * // Lift Buttons
     * //rightButtons.button(3).onTrue(m_LiftSubsystem.goToStartStageCommand());
     * //rightButtons.button(2).onTrue(m_LiftSubsystem.goToCatchStageCommand());
     * //rightButtons.button(1).onTrue(m_LiftSubsystem.goToLiftStageCommand());
     * 
     * joystick.setXChannel(Constants.OperatorConstants.JOYSTICK_X_CHANNEL);
     * //Horizonal axis, algae/coral buttons
     * joystick.setYChannel(Constants.OperatorConstants.JOYSTICK_Y_CHANNEL);
     * //vertical axis, not in use
     */
    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation()) {

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
      //driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      //driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
     /*  driverXbox.b().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
              */
      //driverXbox.start().whileTrue(Commands.none());
      //driverXbox.back().whileTrue(Commands.none());
      //driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      //driverXbox.rightBumper().onTrue(Commands.none());
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

  public Command getHomingCommand() {
    // An example command will be run in autonomous
    return m_ElevatorSubsystem.goToHomeCommand();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public Command scoreCoralCommand(int elevatorStageValue, double coralEncoderValue) {
    return m_ElevatorSubsystem.goToElevatorStageCommand(elevatorStageValue)
        // m_ElevatorSubsystem.goToStagePotentiometerCommand(positionValue)
        // .andThen(m_CoralSubsystem.coralOutputCommand(coralEncoderValue)) ADD BACK,
        // TEMP COMMENT
        .andThen(m_ElevatorSubsystem.goToElevatorStageCommand(Constants.Elevator.CORAL_STAGE_BASE));
    // .andThen(m_ElevatorSubsystem.goToStagePotentiometerCommand(Constants.Elevator.Potentiometer.BASE_STAGE_POT_VALUE));
  }

  public Command collectCoralCommand() {
    return m_ElevatorSubsystem.goToElevatorStageCommand(Constants.Elevator.CORAL_STAGE_BASE)
        // m_ElevatorSubsystem.goToStagePotentiometerCommand(Constants.Elevator.Potentiometer.COLLECT_CORAL_STAGE_POT_VALUE)
        // .andThen(m_CoralSubsystem.coralIntakeCommand()) ADD BACK, TEMP COMMENT
    // .andThen(m_ElevatorSubsystem.goToStagePotentiometerCommand(Constants.Elevator.Potentiometer.BASE_STAGE_POT_VALUE));
  }

}
