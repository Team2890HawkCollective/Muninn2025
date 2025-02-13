package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// NetworkTables if needed, LimelightHelpers is less pain
//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.networktables.NetworkTableInstance;

import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;

public class TargetingSubsytem extends SubsystemBase
{

    private SwerveDrive drivebase;

    public TargetingSubsystem(SwerveDrive driveSystem)
    {
        this.drivebase = driveSystem;
    }

    public Command updatePoseEstimationCommand()
    {
        return runOnce(()->updatePoseEstimation())
    }

    public void initializeLimelight()
    {

    }

    public void updatePoseEstimation()
    {
        double tagId =LimelightHelpers.getFiducialID(Constants.LimeLight.LIMELIGHT_NAME);
        SmartDashboard.putNumber("Visible AprilTag TID", tid);

        Pose2d limelightBotPose = LimelightHelpers.getBotPose2d(Constants.LimeLight.LIMELIGHT_NAME);
        Pose2d drivebaseEstimatedPose = drivebase.m_poseEstimator.getEstimatedPosition();

        SmartDashboard.putNumber("Bot Pose Estimation X", drivebaseEstimatedPose.getX());
        SmartDashboard.putNumber("Bot Pose Estimation Y", drivebaseEstimatedPose.getY());

        drivebase.addVisionMeasurement(limelightBotPose);
        //drivebase.updateEstimatedGlobalPose()

    }

}