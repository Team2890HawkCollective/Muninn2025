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

    public Command updatePoseEstimationCommand()
    {
        return runOnce(()->updatePoseEstimation());
    }

    /**
     * Aligns the bot for coral delivery based on the given AprilTag
     * @param aprilTagId Target AprilTag
     * @return Command
     */
    public Command alignToReefCoralCommand(int aprilTagId){
        return runOnce(()-> alignToReefCoral(aprilTagId));
    }

    /**
     * Aligns the bot for algae pickup based on the given AprilTag
     * @param aprilTagId Target AprilTag
     * @return Command
     */
    public Command alignToReefAlgaeCommand(int aprilTagId){
        return runOnce(()-> alignToReefAlgae(aprilTagId));
    }

    /**
     * Aligns the bot for coral delivery relative to the closest visible Reef AprilTag
     * @param aprilTagId Target AprilTag
     * @return Command
     */
    public Command alignToClosestReefCoralCommand(){
        return runOnce(()-> alignToClosestReefCoral());
    }

    /**
     * Aligns the bot for algae pickup relative to the closest visible Reef AprilTag
     * @param aprilTagId Target AprilTag
     * @return Command
     */
    public Command alignToClosestReefAlgaeCommand(){
        return runOnce(()-> alignToClosestReefAlgae());
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

    public void alignToReefCoral(int aprilTagId){
        
    }

    public void alignToReefAlgae(int aprilTagId){

    }

    public void alignToClosestReefCoral(){
        double tagId =LimelightHelpers.getFiducialID(Constants.LimeLight.LIMELIGHT_NAME);
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if(tagId !=0){
            Pose2d tagPose = LimelightHelpers.getTargetPose3d_RobotSpace(Constants.LimeLight.LIMELIGHT_NAME).toPose2d();
            Pose2d targetPose = new Pose2d((tagPose.getX()-Constants.Coral.DISTANCE_FROM_CENTER),tagPose.getY(),tagPose.getRotation());
            /*
            if(alliance.isPresent()){
                if(alliance.get() == Alliance.Blue){
                    for(int tag : Constants.LimeLight.BlueAprilTags.REEF_APRILTAGS){
                        if(tag == tagId){

                        }
                    }
                }
            }
            if(alliance.isPresent()){
                if(alliance.get() == Alliance.Blue){
                    for(int tag : Constants.LimeLight.BlueAprilTags.REEF_APRILTAGS){
                        if(tag == tagId){

                        }
                    }
                }
            }*/
        }
    }

    public void alignToClosestReefAlgae(){

    }
}