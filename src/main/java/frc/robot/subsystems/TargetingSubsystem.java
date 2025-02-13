package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

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

    public TargetingSubsystem()
    {

    }

    public Command updatePoseEstimationCommand()
    {
        return runOnce(()->updatePoseEstimation());
    }

    public Command alignToReefCoralCommand(int aprilTagId){

    }

    public Command alignToReefAlgaeCommand(int aprilTagId){
        
    }

    public void initializeLimeLight(SwerveDrive driveSystem)
    {
        this.drivebase = driveSystem;
        //Set initial bot orientation
        //Params: Limelight Name, Yaw, Yaw Rate, Pitch, Pitch Rate, Roll, Roll Rate
        LimelightHelpers.SetRobotOrientation(Constants.LimeLight.LIMELIGHT_NAME,drivebase.getYaw(),0,drivebase.getPitch(),0,drivebase.getRoll(),0);
    }

    public void updatePoseEstimation()
    {
        double tagId =LimelightHelpers.getFiducialID(Constants.LimeLight.LIMELIGHT_NAME);

        if (LimelightHelpers.getTV()){
            SmartDashboard.putNumber("Visible AprilTag TID", tagId);
            SmartDashboard.putBoolean("Tracking AprilTag?", true);
        }else{
            SmartDashboard.putNumber("Visible AprilTag TID", 0);
            SmartDashboard.putBoolean("Tracking AprilTag?", false);
        }

        Pose2d limelightBotPose = LimelightHelpers.getBotPose2d(Constants.LimeLight.LIMELIGHT_NAME);
        Pose2d drivebaseEstimatedPose = drivebase.getPose();

        SmartDashboard.putNumber("Bot Pose Estimation X", drivebaseEstimatedPose.getX());
        SmartDashboard.putNumber("Bot Pose Estimation Y", drivebaseEstimatedPose.getY());

        drivebase.addVisionMeasurement(limelightBotPose);
        //drivebase.updateEstimatedGlobalPose()

    }

    public void alignToReefCoral(int aprilTagId){
        
    }

    public void alignToReefAlgae(int aprilTagId){

    }

}