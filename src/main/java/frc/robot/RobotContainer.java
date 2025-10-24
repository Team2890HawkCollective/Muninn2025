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
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.TargetingSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.security.CodeSigner;
import java.util.Set;
import java.util.function.DoubleSupplier;

import org.ejml.dense.block.MatrixOps_MT_DDRB;

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
        // private final LiftSubsystem m_LiftSubsystem = new LiftSubsystem();
        // private final ElevatorSubsystem m_ElevatorSubsystem = new
        // ElevatorSubsystem();
        public final CoralSubsystem m_CoralSubsystem = new CoralSubsystem();
        // public final AlgaeSubsystem m_AlgaeSubsystem = new AlgaeSubsystem();

        private final static CommandJoystick leftButtons = new CommandJoystick(3);
        private final static CommandJoystick rightButtons = new CommandJoystick(2);
        // Replace with CommandPS4Controller or CommandJoystick if needed
        private final static CommandXboxController driverXbox = new CommandXboxController(0);
        private final static CommandXboxController assistantDriverXbox = new CommandXboxController(1);
        // The robot's subsystems and commands are defined here...
        public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                        "swerve"));

        public final TargetingSubsystem m_TargetingSubsystem = new TargetingSubsystem(drivebase, this);

        public ShuffleboardDisplay m_shuffleboardDisplay = new ShuffleboardDisplay();

        public Command defaultCommand = Commands.none();
        // m_TargetingSubsystem.initializeLimeLight();
        /**
         * Converts driver input into a field-relative ChassisSpeeds that is controlled
         * by angular velocity.
         */
        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> driverXbox.getLeftY()*-1, // Joystick Forward Is Negative, the -1 Is Required To Flip This
                        () -> driverXbox.getLeftX()*-1)
                        .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleTranslation(0.8)
                        .allianceRelativeControl(true);

        

        /**
         * Clone's the angular velocity input stream and converts it to a fieldRelative
         * input stream.
         */
        SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                        .withControllerHeadingAxis(() -> driverXbox.getRightX() * -1,
                                        driverXbox::getRightY)
                        .headingWhile(true);

        /**
         * Clone's the angular velocity input stream and converts it to a robotRelative
         * input stream.
         */
        //SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(false)
                        //.allianceRelativeControl(true);

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

                NamedCommands.registerCommand("Launch_Coral",
                                m_CoralSubsystem.shootCoralCommand(Constants.Coral.LAUNCH_CORAL_SPEED)
                                                .andThen(new WaitCommand(1))
                                                .andThen(m_CoralSubsystem.shootCoralCommand(0)));
                // Autonomous Command Registration

                /*
                 * NamedCommands.registerCommand("Coral_Level_2_HalfCycle",
                 * m_ElevatorSubsystem.goToElevatorStageCommand(2)
                 * .andThen(new WaitCommand(Constants.Coral.RotationMotor.ROTATE_DELAY))
                 * .andThen(m_CoralSubsystem.coralOutputCommand()));
                 * NamedCommands.registerCommand("Coral_Level_3_HalfCycle",
                 * m_ElevatorSubsystem.goToElevatorStageCommand(3)
                 * .andThen(new WaitCommand(Constants.Coral.RotationMotor.ROTATE_DELAY))
                 * .andThen(m_CoralSubsystem.coralOutputCommand()));
                 * NamedCommands.registerCommand("Coral_Level_4_HalfCycle",
                 * m_ElevatorSubsystem.goToElevatorStageCommand(4)
                 * .andThen(new WaitCommand(Constants.Coral.RotationMotor.ROTATE_DELAY))
                 * .andThen(m_CoralSubsystem.coralL4OutputCommand()));
                 * NamedCommands.registerCommand("Algae_Level_1_HalfCycle",
                 * m_ElevatorSubsystem.goToElevatorStageCommand(5));
                 * NamedCommands.registerCommand("Algae_Level_2_HalfCycle",
                 * m_ElevatorSubsystem.goToElevatorStageCommand(6));
                 * 
                 * NamedCommands.registerCommand("CollectAlgaePos",
                 * m_AlgaeSubsystem.AlgaeOutputCommand());
                 * NamedCommands.registerCommand("CarryAlgaePos",
                 * m_AlgaeSubsystem.AlgaeStartCommand());
                 * 
                 * NamedCommands.registerCommand("homeElevator",
                 * m_ElevatorSubsystem.goToHomeCommand());
                 * NamedCommands.registerCommand("openCoralServo",
                 * m_CoralSubsystem.servoRotateToOpen());
                 * NamedCommands.registerCommand("coralIntake",
                 * m_CoralSubsystem.coralIntakeCommand());
                 * NamedCommands.registerCommand("intakeAlgae",
                 * m_AlgaeSubsystem.moveInputAlgaeWheelsCommand());
                 */
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
                //Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
                Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
                Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
                Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
                Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);


                driverXbox.leftTrigger().whileTrue(m_CoralSubsystem.shootCoralCommand(-.2))
                                .onFalse(m_CoralSubsystem.shootCoralCommand(0));
                driverXbox.rightTrigger()
                                .whileTrue(m_CoralSubsystem.shootCoralCommand(Constants.Coral.LAUNCH_CORAL_SPEED))
                                .onFalse(m_CoralSubsystem.shootCoralCommand(0));


                driverXbox.x().onTrue(drivebase.ZERO_GYRO());
                // if(Constants.ShuffleboardConstants.CONTROL_MODE.equalsIgnoreCase("manual")){
                // if(ShuffleboardDisplay.getControlModeChoice().equalsIgnoreCase("manual")){
                // Assistant Driver Manual Control
                // assistantDriverXbox.y().whileTrue(m_ElevatorSubsystem.moveElevatorUpCommand())
                // .onFalse(m_ElevatorSubsystem.stopElevatorMotorCommand()); // Manual Elevator
                // Up
                // assistantDriverXbox.a().whileTrue(m_ElevatorSubsystem.moveElevatorDownCommand())
                // .onFalse(m_ElevatorSubsystem.stopElevatorMotorCommand()); // Manual Elevator
                // Down
                /*
                 * assistantDriverXbox.y().whileTrue(m_AlgaeSubsystem.
                 * testManualAlgaeRotateUpCommand()).onFalse(m_AlgaeSubsystem.
                 * stopRotationMotorCommand());
                 * assistantDriverXbox.a().whileTrue(m_AlgaeSubsystem.
                 * testManualAlgaeRotateDownCommand()).onFalse(m_AlgaeSubsystem.
                 * stopRotationMotorCommand());
                 * assistantDriverXbox.b().onTrue(m_ElevatorSubsystem.stopElevatorMotorCommand()
                 * );
                 * assistantDriverXbox.povLeft().onTrue(m_AlgaeSubsystem.AlgaeStartCommand());
                 * // Algae Start Position
                 * assistantDriverXbox.povUp().onTrue(m_AlgaeSubsystem.AlgaeCarryCommand()); //
                 * Algae Carry Position
                 * assistantDriverXbox.povRight().onTrue(m_AlgaeSubsystem.AlgaeOutputCommand());
                 * // Algae Output Position
                 * assistantDriverXbox.povDown()
                 * .onTrue(m_AlgaeSubsystem.rotateToPositionCommand(Constants.Algae.Rotation.
                 * PROCESSOR_ENCODER_VALUE));
                 * assistantDriverXbox.leftBumper()
                 * .onTrue(m_CoralSubsystem
                 * .rotateToPositionCommand(Constants.Coral.RotationMotor.
                 * START_POSITION_ENCODER_VALUE));
                 * assistantDriverXbox.rightBumper()
                 * .onTrue(m_CoralSubsystem
                 * .rotateToPositionCommand(Constants.Coral.RotationMotor.
                 * SCORE_POSITION_ENCODER_VALUE));
                 */
                // assistantDriverXbox.leftTrigger().onTrue(m_LiftSubsystem.moveToCatchPositionCommand())
                // .onFalse(m_LiftSubsystem.stopLiftMotorCommand());
                // assistantDriverXbox.leftTrigger().onTrue(m_LiftSubsystem.moveToStartPositionCommand())
                // .onFalse(m_LiftSubsystem.stopLiftMotorCommand());
                //assistantDriverXbox.start()
                                //.onTrue(Commands.defer(() -> m_TargetingSubsystem.stringRunAuton(),
                                                //Set.of(m_AlgaeSubsystem, m_TargetingSubsystem, m_CoralSubsystem,
                                                                //drivebase, m_ElevatorSubsystem)));

                // } else {
                // Elevator Stage Buttons
                /*
                 * leftButtons.button(1)
                 * .onTrue(m_ElevatorSubsystem.goToElevatorStageCommand(6)
                 * .andThen(new WaitCommand(Constants.Coral.RotationMotor.ROTATE_DELAY))
                 * .andThen(m_AlgaeSubsystem.AlgaeOutputCommand())); // Algae L3
                 * leftButtons.button(2)
                 * .onTrue(m_ElevatorSubsystem.goToElevatorStageCommand(5)
                 * .andThen(new WaitCommand(Constants.Coral.RotationMotor.ROTATE_DELAY))
                 * .andThen(m_AlgaeSubsystem.AlgaeOutputCommand())); // Algae L2
                 * leftButtons.button(3)
                 * .onTrue(m_ElevatorSubsystem.goToElevatorStageCommand(4)
                 * .andThen(m_AlgaeSubsystem.AlgaeStartCommand())
                 * .andThen(new WaitCommand(Constants.Coral.RotationMotor.ROTATE_DELAY))
                 * .andThen(m_CoralSubsystem.coralL4OutputCommand())); // Coral L4
                 * .andThen(m_CoralSubsystem.coralL4OutputCommand())); // Coral L4
                 * leftButtons.button(4)
                 * .onTrue(m_ElevatorSubsystem.goToElevatorStageCommand(3)
                 * .andThen(new WaitCommand(Constants.Coral.RotationMotor.ROTATE_DELAY))
                 * .andThen(m_CoralSubsystem.coralOutputCommand())); // Coral L3
                 * leftButtons.button(5)
                 * .onTrue(m_ElevatorSubsystem.goToElevatorStageCommand(2)
                 * .andThen(m_AlgaeSubsystem.AlgaeStartCommand())
                 * .andThen(new WaitCommand(Constants.Coral.RotationMotor.ROTATE_DELAY))
                 * .andThen(m_CoralSubsystem.coralOutputCommand())); // Coral L2; Skips Coral L1
                 * leftButtons.button(6)
                 * .onTrue(m_AlgaeSubsystem.AlgaeStartCommand()
                 * .andThen(m_CoralSubsystem.servoRotateToClosed())
                 * .andThen(m_CoralSubsystem
                 * .rotateToPositionCommand(Constants.Coral.RotationMotor.
                 * START_POSITION_ENCODER_VALUE))
                 * .andThen(m_ElevatorSubsystem.goToHomeCommand())); // Elevator All The Way
                 * Down
                 * 
                 * // Elevator Manual Control
                 * leftButtons.axisGreaterThan(1,
                 * 0.3).toggleOnTrue(m_ElevatorSubsystem.moveElevatorUpCommand())
                 * .toggleOnFalse(m_ElevatorSubsystem.stopElevatorMotorCommand().andThen(
                 * m_ElevatorSubsystem.holdPositionCommand()));
                 * leftButtons.axisLessThan(1, -0.3).toggleOnTrue(//
                 * m_AlgaeSubsystem.AlgaeStartCommand().andThen(
                 * m_CoralSubsystem.servoRotateToClosed()
                 * .andThen(m_ElevatorSubsystem.moveElevatorDownCommand())
                 * .andThen(m_ElevatorSubsystem.holdPositionCommand()))// )
                 * .toggleOnFalse(m_ElevatorSubsystem.stopElevatorMotorCommand());
                 * rightButtons.button(7)
                 * .onTrue(Commands.defer(() -> m_ElevatorSubsystem.holdPositionCommand(),
                 * Set.of(m_ElevatorSubsystem))); // Holds
                 * // current
                 * // position
                 * 
                 * // Manual Coral Tilt
                 * rightButtons.button(8).onTrue(
                 * m_CoralSubsystem.rotateToPositionCommand(Constants.Coral.RotationMotor.
                 * START_POSITION_ENCODER_VALUE));
                 * rightButtons.button(9).onTrue(
                 * m_CoralSubsystem.rotateToPositionCommand(Constants.Coral.RotationMotor.
                 * SCORE_POSITION_ENCODER_VALUE));
                 * rightButtons.button(3).onTrue(
                 * m_CoralSubsystem.rotateToPositionCommand(Constants.Coral.RotationMotor.
                 * L4_CORAL_ENCODER_VALUE));
                 * 
                 * // Algae
                 * rightButtons.axisGreaterThan(0,
                 * 0.3).onTrue(m_AlgaeSubsystem.AlgaeStartCommand()); // Start Position
                 * rightButtons.axisLessThan(0,
                 * -0.3).onTrue(m_AlgaeSubsystem.AlgaeOutputCommand());//.andThen(
                 * m_AlgaeSubsystem.moveInputAlgaeWheelsCommand())); // Carry Position
                 * rightButtons.axisGreaterThan(1,
                 * 0.3).toggleOnTrue(m_AlgaeSubsystem.manualAlgaeUpCommand())
                 * .toggleOnFalse(m_AlgaeSubsystem.stopRotationMotorCommand()); // Manual Up
                 * rightButtons.axisLessThan(1,
                 * -0.3).toggleOnTrue(m_AlgaeSubsystem.manualAlgaeDownCommand())
                 * .toggleOnFalse(m_AlgaeSubsystem.stopRotationMotorCommand()); // Manual Down
                 * \
                 * /*
                 * // 4 Direction Joystick
                 * rightButtons.axisGreaterThan(1,
                 * 0.3).onTrue(m_AlgaeSubsystem.AlgaeCarryCommand()); // Carry Position
                 * rightButtons.axisLessThan(1,
                 * -0.3).onTrue(m_AlgaeSubsystem.rotateToPositionCommand(Constants.Algae.
                 * Rotation.PROCESSOR_ENCODER_VALUE)); // Processor Position
                 * rightButtons.axisGreaterThan(0,
                 * 0.3).onTrue(m_AlgaeSubsystem.rotateToPositionCommand(Constants.Algae.Rotation
                 * .START_POSITION_ENCODER_VALUE)); // Start Position
                 * rightButtons.axisGreaterThan(0,
                 * -0.3).onTrue(m_AlgaeSubsystem.AlgaeOutputCommand()); // Output Command
                 */

                // Assistant Driver Alignment Buttons
                rightButtons.button(10)
                                .onTrue(m_TargetingSubsystem.autoAlignmentCommand("left")
                                                .andThen(() -> m_TargetingSubsystem.enableVisionUpdates())); // Re-enable
                                                                                                             // vision
                                                                                                             // after
                                                                                                             // the
                                                                                                             // pathfind
                                                                                                             // is
                                                                                                             // complete.
                // .andThen(Commands.runOnce(()->Led.turnOffAlignmentLights())));
                rightButtons.button(11)
                                .onTrue(m_TargetingSubsystem.autoAlignmentCommand("center")
                                                .andThen(() -> m_TargetingSubsystem.enableVisionUpdates())); // Re-enable
                                                                                                             // vision
                                                                                                             // after
                                                                                                             // the
                                                                                                             // pathfind
                                                                                                             // is
                                                                                                             // complete.
                // .andThen(Commands.runOnce(()->Led.turnOffAlignmentLights())));
                rightButtons.button(12)
                                .onTrue(m_TargetingSubsystem.autoAlignmentCommand("right")
                                                .andThen(() -> m_TargetingSubsystem.enableVisionUpdates())); // Re-enable
                                                                                                             // vision
                                                                                                             // after
                                                                                                             // the
                                                                                                             // pathfind
                                                                                                             // is
                                                                                                             // complete.
                // .andThen(Commands.runOnce(()->Led.turnOffAlignmentLights())));

                // Driver Alignment Buttons
                driverXbox.povLeft()
                                .onTrue(m_TargetingSubsystem.autoAlignmentCommand("left")
                                                .andThen(() -> m_TargetingSubsystem.enableVisionUpdates())); // Re-enable
                                                                                                             // vision
                                                                                                             // after
                                                                                                             // the
                                                                                                             // pathfind
                                                                                                             // is
                                                                                                             // complete.
                // .andThen(Commands.runOnce(()->Led.turnOffAlignmentLights())));
                driverXbox.povUp()
                                .onTrue(m_TargetingSubsystem.autoAlignmentCommand("center")
                                                .andThen(() -> m_TargetingSubsystem.enableVisionUpdates())); // Re-enable
                                                                                                             // vision
                                                                                                             // after
                                                                                                             // the
                                                                                                             // pathfind
                                                                                                             // is
                                                                                                             // complete.
                // .andThen(Commands.runOnce(()->Led.turnOffAlignmentLights())));
                driverXbox.povRight()
                                .onTrue(m_TargetingSubsystem.autoAlignmentCommand("right")
                                                .andThen(() -> m_TargetingSubsystem.enableVisionUpdates())); // Re-enable
                                                                                                             // vision
                                                                                                             // after
                                                                                                             // the
                                                                                                             // pathfind
                                                                                                             // is
                                                                                                             // complete.
                // .andThen(Commands.runOnce(()->Led.turnOffAlignmentLights())));

                // Override
                rightButtons.button(6).onTrue(manualOverrideCommand());
                driverXbox.povDown().onTrue(manualOverrideCommand());

                // rightButtons.button(4).onTrue(m_CoralSubsystem.servoRotateToOpen()); //
                // Assist Driver Open Coral Servo
                // rightButtons.button(5).onTrue(m_CoralSubsystem.servoRotateToClosed()); //
                // Asist Driver Close Coral Servo

                // Lift Position Buttons
                // rightButtons.button(6).onTrue(m_LiftSubsystem.moveToPositionCommand(Constants.Lift.catchPosition));
                // rightButtons.button(7).onTrue(m_LiftSubsystem.moveToPositionCommand(Constants.Lift.liftPosition));
                // rightButtons.button(8).onTrue(m_LiftSubsystem.lockRatchetCommand());
                // rightButtons.button(9).onTrue(m_LiftSubsystem.retractRatchetCommand());

                // }

                // rightButtons.button(6).onTrue(m_TargetingSubsystem.pathfindTest());
                // Driver Controls
                // driverXbox.leftBumper().onTrue(m_CoralSubsystem.servoRotateToClosed()); //
                // Close Coral Servo
                // driverXbox.rightBumper().onTrue(m_CoralSubsystem.servoRotateToOpen());//
                // .andThen(new
                // WaitCommand(1)).andThen(m_CoralSubsystem.rotateToPositionCommand(Constants.Coral.RotationMotor.START_POSITION_ENCODER_VALUE)));
                /*
                 * // // Open Coral Servo
                 * 
                 * driverXbox.leftTrigger().whileTrue(m_AlgaeSubsystem.
                 * moveInputAlgaeWheelsCommand());
                 * //.onFalse(m_AlgaeSubsystem.stopAlgaeWheelsCommand()); // Intake Algae
                 * driverXbox.rightTrigger().whileTrue(m_AlgaeSubsystem.
                 * moveOutputAlgaeWheelsCommand())
                 * .onFalse(m_AlgaeSubsystem.stopAlgaeWheelsCommand()); // Output Algae
                 * 
                 * driverXbox.b().onTrue(m_LiftSubsystem.retractRatchetCommand());
                 * driverXbox.x().onTrue(m_LiftSubsystem.lockRatchetCommand());
                 * driverXbox.y().onTrue(// m_LiftSubsystem.retractRatchetCommand())
                 * // .whileTrue(
                 * m_LiftSubsystem.moveLiftUpCommand())
                 * .onFalse(m_LiftSubsystem.stopLiftMotorCommand()
                 * .andThen(m_LiftSubsystem.lockRatchetCommand())); // Manual Lift Up
                 * driverXbox.a().onTrue(m_LiftSubsystem.retractRatchetCommand())
                 * .whileTrue(m_LiftSubsystem.moveLiftDownCommand() // Manual Lift Down
                 * // .andThen(m_AlgaeSubsystem.algaeLiftCommand())
                 * ).onFalse(m_LiftSubsystem.stopLiftMotorCommand()
                 * // .andThen(m_LiftSubsystem.lockRatchetCommand())
                 * );
                 */
                // Driver Alignment Controls
                // driverXbox.povLeft().onTrue(m_TargetingSubsystem.autoAlignmentCommand("left"));
                // driverXbox.povUp().onTrue(m_TargetingSubsystem.autoAlignmentCommand("center"));
                // driverXbox.povRight().onTrue(m_TargetingSubsystem.autoAlignmentCommand("right"));

                if (RobotBase.isSimulation()) {
                        drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
                } else {
                        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
                }

                if (Robot.isSimulation()) {

                        driverXbox.start()
                                        .onTrue(Commands.runOnce(() -> drivebase
                                                        .resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
                        driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

                }
                if (DriverStation.isTest()) {
                        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command
                                                                                         // above!

                        driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
                        driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
                        driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
                        driverXbox.back().whileTrue(drivebase.centerModulesCommand());
                        driverXbox.leftBumper().onTrue(Commands.none());
                        driverXbox.rightBumper().onTrue(Commands.none());
                } else {
                        driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
                        /*
                         * driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
                         * driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
                         * driverXbox.b().whileTrue(
                         * drivebase.driveToPose(
                         * new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
                         * driverXbox.start().whileTrue(Commands.none());
                         * driverXbox.back().whileTrue(Commands.none());
                         * driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock,
                         * drivebase).repeatedly());
                         * driverXbox.rightBumper().onTrue(Commands.none());
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

        /*
         * public Command getHomingCommand() {
         * // An example command will be run in autonomous
         * return m_ElevatorSubsystem.goToHomeCommand();
         * }
         */
        public void setMotorBrake(boolean brake) {
                drivebase.setMotorBrake(brake);
        }

        public static CommandJoystick getLeftButtons() {
                return leftButtons;
        }

        public static CommandJoystick getRightButtons() {
                return rightButtons;
        }

        public static CommandXboxController getDriverXbox() {
                return driverXbox;
        }

        public static CommandXboxController getAssistantDriverXbox() {
                return assistantDriverXbox;
        }

        public Command manualOverrideCommand() {
                return Commands.runOnce(() -> manualOverride());
        }

        public void manualOverride() {
                CommandScheduler.getInstance().cancelAll();
                // m_ElevatorSubsystem.stopElevatorMotor();
                // m_LiftSubsystem.stopLiftMotor();
                // m_AlgaeSubsystem.stopWheels();
                // m_AlgaeSubsystem.stopRotationMotor();
        }

        public Command enableDriverControlOverride() {
                return Commands.runOnce(() -> drivebase.setDefaultCommand(Commands.none()));
        }

        public Command disableDriverControlOverride() {
                return Commands.runOnce(() -> drivebase.setDefaultCommand(defaultCommand));
        }
}
