package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// NetworkTables if needed, LimelightHelpers is less pain
//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.networktables.NetworkTableInstance;

import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;

public class TargetingSubsystem extends SubsystemBase
{

    private SwerveDrive drivebase;

    public TargetingSubsystem()
    {
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updatePoseEstimation();
    }

    public Command updatePoseEstimationCommand()
    {
        return runOnce(()->updatePoseEstimation());
    }

    public Command pathfindToCoralBranchCommand(String branch){
        return runOnce(()->pathfindToCoralBranch(branch));
    }

    public void initializeLimeLight(SwerveDrive driveSystem)
    {
        this.drivebase = driveSystem;
        //Set initial bot orientation
        //Params: Limelight Name, Yaw, Yaw Rate, Pitch, Pitch Rate, Roll, Roll Rate
        LimelightHelpers.SetRobotOrientation(Constants.LimeLight.LIMELIGHT_NAME,drivebase.getYaw().getDegrees(),0,drivebase.getPitch().getDegrees(),0,drivebase.getRoll().getDegrees(),0);
    }

    public void updatePoseEstimation()
    {
        double tagId =LimelightHelpers.getFiducialID(Constants.LimeLight.LIMELIGHT_NAME);

        if(tagId != 0){
            if (LimelightHelpers.getTV(Constants.LimeLight.LIMELIGHT_NAME)){
                SmartDashboard.putNumber("Visible AprilTag TID", tagId);
                SmartDashboard.putBoolean("Tracking AprilTag?", true);
            }else{
                SmartDashboard.putNumber("Visible AprilTag TID", 0);
                SmartDashboard.putBoolean("Tracking AprilTag?", false);
            }

            LimelightHelpers.PoseEstimate limelightBotPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LimeLight.LIMELIGHT_NAME);
            Pose2d drivebaseEstimatedPose = drivebase.getPose();

            SmartDashboard.putNumber("Bot Pose Estimation X", drivebaseEstimatedPose.getX());
            SmartDashboard.putNumber("Bot Pose Estimation Y", drivebaseEstimatedPose.getY());

            drivebase.addVisionMeasurement(limelightBotPoseEstimate.pose,limelightBotPoseEstimate.timestampSeconds);
            //drivebase.updateEstimatedGlobalPose()
        }
    }

    public void pathfindToCoralBranch(String branch){
        
    } 
}