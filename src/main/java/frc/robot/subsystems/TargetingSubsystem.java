package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.Arrays;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// NetworkTables if needed, LimelightHelpers is less pain
//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.networktables.NetworkTableInstance;

import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;

public class TargetingSubsystem extends SubsystemBase {

    private SwerveDrive drivebase;

    public TargetingSubsystem() {
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updatePoseEstimation();
    }

    public Command updatePoseEstimationCommand() {
        return runOnce(() -> updatePoseEstimation());
    }

    public Command autoAlignmentCommand(String location) {
        return autoAlignment(location);
    }

    public Command pathfindTest() {
        PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(
                Constants.LimeLight.BlueReefPositions.CoralPoses.G,
                constraints,
                0.0 // Goal end velocity in meters/sec
        ).andThen(AutoBuilder.pathfindToPose(
                Constants.LimeLight.RedReefPositions.CoralPoses.A,
                constraints,
                0.0 // Goal end velocity in meters/sec
        ));
    }

    public void initializeLimeLight(SwerveDrive driveSystem) {
        this.drivebase = driveSystem;
        LimelightHelpers.SetFiducialIDFiltersOverride(Constants.LimeLight.LIMELIGHT_NAME, Constants.LimeLight.ALL_REEF_APRILTAGS); // Filter Out Non-Reef tags
        // Set initial bot orientation
        // Params: Limelight Name, Yaw, Yaw Rate, Pitch, Pitch Rate, Roll, Roll Rate
        LimelightHelpers.SetRobotOrientation(Constants.LimeLight.LIMELIGHT_NAME, drivebase.getYaw().getDegrees(), 0,
                drivebase.getPitch().getDegrees(), 0, drivebase.getRoll().getDegrees(), 0);
    }

    public void updatePoseEstimation() {
        double tagId = LimelightHelpers.getFiducialID(Constants.LimeLight.LIMELIGHT_NAME);

        if (tagId != 0) {
            if (LimelightHelpers.getTV(Constants.LimeLight.LIMELIGHT_NAME)) {
                SmartDashboard.putNumber("Visible AprilTag TID", tagId);
                SmartDashboard.putBoolean("Tracking AprilTag?", true);
            } else {
                SmartDashboard.putNumber("Visible AprilTag TID", 0);
                SmartDashboard.putBoolean("Tracking AprilTag?", false);
            }

            LimelightHelpers.PoseEstimate limelightBotPoseEstimate = LimelightHelpers
                    .getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LimeLight.LIMELIGHT_NAME);
            Pose2d drivebaseEstimatedPose = drivebase.getPose();

            SmartDashboard.putNumber("Bot Pose Estimation X", drivebaseEstimatedPose.getX());
            SmartDashboard.putNumber("Bot Pose Estimation Y", drivebaseEstimatedPose.getY());

            drivebase.addVisionMeasurement(limelightBotPoseEstimate.pose, limelightBotPoseEstimate.timestampSeconds);
            // drivebase.updateEstimatedGlobalPose()
        }
    }

    public Command autoAlignment(String location) {
        PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        int tagId = (int) LimelightHelpers.getFiducialID(Constants.LimeLight.LIMELIGHT_NAME);

        Pose2d targetPose = new Pose2d();

        if (tagId != 0) {
            // Red Reef Tags
            if (tagId == 7) {
                switch (location.toLowerCase()) {
                    case "left":
                        targetPose = Constants.LimeLight.RedReefPositions.CoralPoses.A;
                    case "center":
                        targetPose = Constants.LimeLight.RedReefPositions.AlgaePoses.THREE;
                    case "right":
                        targetPose = Constants.LimeLight.RedReefPositions.CoralPoses.B;
                }
            }
            if (tagId == 8) {
                switch (location.toLowerCase()) {
                    case "left":
                        targetPose = Constants.LimeLight.RedReefPositions.CoralPoses.C;
                    case "center":
                        targetPose = Constants.LimeLight.RedReefPositions.AlgaePoses.ONE;
                    case "right":
                        targetPose = Constants.LimeLight.RedReefPositions.CoralPoses.D;
                }
            }
            if (tagId == 9) {
                switch (location.toLowerCase()) {
                    case "left":
                        targetPose = Constants.LimeLight.RedReefPositions.CoralPoses.E;
                    case "center":
                        targetPose = Constants.LimeLight.RedReefPositions.AlgaePoses.ELEVEN;
                    case "right":
                        targetPose = Constants.LimeLight.RedReefPositions.CoralPoses.F;
                }
            }
            if (tagId == 10) {
                switch (location.toLowerCase()) {
                    case "left":
                        targetPose = Constants.LimeLight.RedReefPositions.CoralPoses.G;
                    case "center":
                        targetPose = Constants.LimeLight.RedReefPositions.AlgaePoses.NINE;
                    case "right":
                        targetPose = Constants.LimeLight.RedReefPositions.CoralPoses.H;
                }
            }
            if(tagId == 11){
                switch (location.toLowerCase()) {
                    case "left":
                        targetPose = Constants.LimeLight.RedReefPositions.CoralPoses.I;
                    case "center":
                        targetPose = Constants.LimeLight.RedReefPositions.AlgaePoses.SEVEN;
                    case "right":
                        targetPose = Constants.LimeLight.RedReefPositions.CoralPoses.J;
                }
            }
            if(tagId == 6){
                switch (location.toLowerCase()) {
                    case "left":
                        targetPose = Constants.LimeLight.RedReefPositions.CoralPoses.K;
                    case "center":
                        targetPose = Constants.LimeLight.RedReefPositions.AlgaePoses.FIVE;
                    case "right":
                        targetPose = Constants.LimeLight.RedReefPositions.CoralPoses.L;
                }
            }

            // Blue Reef Tags
            if (tagId == 18) {
                switch (location.toLowerCase()) {
                    case "left":
                        targetPose = Constants.LimeLight.BlueReefPositions.CoralPoses.A;
                    case "center":
                        targetPose = Constants.LimeLight.BlueReefPositions.AlgaePoses.THREE;
                    case "right":
                        targetPose = Constants.LimeLight.BlueReefPositions.CoralPoses.B;
                }
            }
            if (tagId == 17) {
                switch (location.toLowerCase()) {
                    case "left":
                        targetPose = Constants.LimeLight.BlueReefPositions.CoralPoses.C;
                    case "center":
                        targetPose = Constants.LimeLight.BlueReefPositions.AlgaePoses.ONE;
                    case "right":
                        targetPose = Constants.LimeLight.BlueReefPositions.CoralPoses.D;
                }
            }
            if (tagId ==22) {
                switch (location.toLowerCase()) {
                    case "left":
                        targetPose = Constants.LimeLight.BlueReefPositions.CoralPoses.E;
                    case "center":
                        targetPose = Constants.LimeLight.BlueReefPositions.AlgaePoses.ELEVEN;
                    case "right":
                        targetPose = Constants.LimeLight.BlueReefPositions.CoralPoses.F;
                }
            }
            if (tagId == 21) {
                switch (location.toLowerCase()) {
                    case "left":
                        targetPose = Constants.LimeLight.BlueReefPositions.CoralPoses.G;
                    case "center":
                        targetPose = Constants.LimeLight.BlueReefPositions.AlgaePoses.NINE;
                    case "right":
                        targetPose = Constants.LimeLight.BlueReefPositions.CoralPoses.H;
                }
            }
            if(tagId == 20){
                switch (location.toLowerCase()) {
                    case "left":
                        targetPose = Constants.LimeLight.BlueReefPositions.CoralPoses.I;
                    case "center":
                        targetPose = Constants.LimeLight.BlueReefPositions.AlgaePoses.SEVEN;
                    case "right":
                        targetPose = Constants.LimeLight.BlueReefPositions.CoralPoses.J;
                }
            }
            if(tagId == 19){
                switch (location.toLowerCase()) {
                    case "left":
                        targetPose = Constants.LimeLight.BlueReefPositions.CoralPoses.K;
                    case "center":
                        targetPose = Constants.LimeLight.BlueReefPositions.AlgaePoses.FIVE;
                    case "right":
                        targetPose = Constants.LimeLight.BlueReefPositions.CoralPoses.L;
                }
            }
        return AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0 // Goal end velocity in meters/sec
        );
    }else{
        return null;
    }
    }

    // Function return true if given element
    // found in array
    private static boolean check(Integer[] arr, int toCheckValue) {
        // check if the specified element
        // is present in the array or not
        // using contains() method
        boolean test = Arrays.asList(arr)
                .contains(toCheckValue);

        // Print the result
        return test;
    }
}