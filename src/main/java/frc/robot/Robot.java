// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import javax.lang.model.type.NullType;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.TargetingSubsystem;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this
 * class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {

  private static Robot instance;
  private Command m_autonomousCommand;

  public RobotContainer m_robotContainer;

  private CoralSubsystem m_CoralSubsystem;
  private ShuffleboardDisplay m_shuffleboardDisplay;
  private TargetingSubsystem m_TargetingSubsystem;
  private String m_choosenAutoMode;

  private Timer disabledTimer;

  public Robot() {
    instance = this;
  };

  public static Robot getInstance() {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    m_shuffleboardDisplay = new ShuffleboardDisplay();
    m_shuffleboardDisplay.initiateDisplay();
    m_shuffleboardDisplay.initializeAutoChooser();

    // close servo on startup
    m_CoralSubsystem = new CoralSubsystem();
    m_CoralSubsystem.servoRotateToClosed();

    m_robotContainer.m_TargetingSubsystem.disableVisionUpdates(); // Disable Vision on initialization

    // Create a timer to disable motor brake a few seconds after disable. This will
    // let the robot stop
    // immediately when disabled, but then also let it be pushed more
    disabledTimer = new Timer();

    // Turn On LEDs
    Led.initLED();
    //Led.setColor(Color.kGold);
    //Led.setColorBreathe(Color.kTeal, Color.kPink);

    if (isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    new Thread(() -> {
      UsbCamera camera = CameraServer.startAutomaticCapture();
      camera.setResolution(640, 480);

      CvSink cvSink = CameraServer.getVideo();
      CvSource outputStream = CameraServer.putVideo("Blur", 640, 480);

      Mat source = new Mat();
      Mat output = new Mat();

      while (!Thread.interrupted()) {
        if (cvSink.grabFrame(source) == 0) {
          continue;
        }
        Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
        outputStream.putFrame(output);
      }
    }).start();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    //Led.updatePeriodically();
    CommandScheduler.getInstance().run();
    // m_CoralSubsystem.updateLED();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll(); // Kills all commands when disabled.
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic() {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME)) {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
      disabledTimer.reset();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_robotContainer.setMotorBrake(true);

    // Zero Encoders
    m_robotContainer.m_CoralSubsystem.zeroEncoder(); // Coralp OP\
    m_robotContainer.m_AlgaeSubsystem.zeroEncoder(); // Algae
    m_robotContainer.drivebase.zeroGyroWithAlliance();

    m_robotContainer.m_TargetingSubsystem.disableVisionUpdates();

    // m_robotContainer.getHomingCommand().schedule();
    Command choosenAutoMode = m_shuffleboardDisplay.getAutonomousChoice();
    SmartDashboard.putData("Selected Auto Mode", choosenAutoMode);
    m_autonomousCommand = choosenAutoMode;
    //m_autonomousCommand = m_shuffleboardDisplay.getAutonomousCommand();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    m_robotContainer.m_TargetingSubsystem.enableVisionUpdates();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    } else {
      CommandScheduler.getInstance().cancelAll();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    if (DriverStation.isFMSAttached()) { // The timer acts differently depending on if FMS is controlling it
      if (DriverStation.getMatchTime() <= 20) {
       // Led.setColorRainbow();
      }
    } else {
      if (DriverStation.getMatchTime() >= 115) // In teleop & auto (NO FMS) the timer counts UP.
      {
        //Led.setColorRainbow();
      }
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    // m_shuffleboardDisplay.testingPIDTab(m_robotContainer.getSwerveDriveInfo());
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit() {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic() {
  }
}
