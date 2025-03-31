package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;

import org.ejml.dense.row.linsol.qr.LinearSolverQr_CDRM;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.RectanglePoseArea;
// NetworkTables if needed, LimelightHelpers is less pain
//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;

public class TargetingSubsystem extends SubsystemBase {

    private SwerveSubsystem swerveSub;
    private SwerveDrive drivebase;

    private Pose2d lastPose;

    StructPublisher<Pose2d> startPosePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("StartingPose", Pose2d.struct).publish();
    StructPublisher<Pose2d> targetPosePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("TargetPose", Pose2d.struct).publish();
    StructPublisher<Pose2d> visionPoseEstimatorPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("visionPoseEstimator", Pose2d.struct).publish();
    StructPublisher<Pose2d> currentSwervePose = NetworkTableInstance.getDefault()
            .getStructTopic("currentSwervePose", Pose2d.struct).publish();
    StructPublisher<Pose2d> autoBuilderPose = NetworkTableInstance.getDefault()
            .getStructTopic("autoBuilderPose", Pose2d.struct).publish();
    StructPublisher<Pose2d> limelightPosePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("limelightPose", Pose2d.struct).publish();

    private final Field2d m_field = new Field2d();

    // The fieldBoundry is gotten from the bottom left, and upper right corners.
    // Bottom Left is always (0,0)
    // Upper Right is the dimensions of the field. I found that this is in meters by
    // comparing the values provided in the example to last year's field dimensions.
    private final RectanglePoseArea fieldBoundary = new RectanglePoseArea(new Translation2d(0, 0),
            new Translation2d(17.55, 8.05));

    private final SwerveDrivePoseEstimator visionPoseEstimator;

    public TargetingSubsystem(SwerveSubsystem driveSystem) {
        this.drivebase = driveSystem.getSwerveDrive();
        this.swerveSub = driveSystem;
        LimelightHelpers.SetFiducialIDFiltersOverride(Constants.LimeLight.LIMELIGHT_NAME,
                Constants.LimeLight.ALL_REEF_APRILTAGS); // Filter Out Non-Reef tags

        this.visionPoseEstimator = new SwerveDrivePoseEstimator(
                drivebase.kinematics,
                drivebase.getGyro().getRotation3d().toRotation2d(),
                new SwerveModulePosition[] {
                        drivebase.getModulePositions()[0], // Front Left
                        drivebase.getModulePositions()[0], // Front Right
                        drivebase.getModulePositions()[0], // Back Left
                        drivebase.getModulePositions()[0] // Back Right
                // m_frontLeft.getPosition(),
                // m_frontRight.getPosition(),
                // m_backLeft.getPosition(),
                // m_backRight.getPosition()
                },
                Pose2d.kZero,
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
        
        lastPose = drivebase.getPose();

        // Set initial bot orientation
        // Params: Limelight Name, Yaw, Yaw Rate, Pitch, Pitch Rate, Roll, Roll Rate
        // LimelightHelpers.SetRobotOrientation(Constants.LimeLight.LIMELIGHT_NAME,
        // drivebase.getYaw().getDegrees(), 0,
        // drivebase.getPitch().getDegrees(), 0, drivebase.getRoll().getDegrees(), 0);
        SmartDashboard.putData("Field", m_field); // Add the field widget (Not working rn)

        SmartDashboard.putNumber("Target Pose X", -1); // Add Target Pose X display. set to arbitrary -1.
        SmartDashboard.putNumber("Target Pose Y", -1); // Add Target Pose Y display. set to arbitrary -1.
        SmartDashboard.putNumber("Tag Pose X", -1); // Add the Tag Pose X display, set to arbitrary -1.
        SmartDashboard.putNumber("Tag Pose Y", -1); // Add the Tag Pose Y display, set to arbitrary -1.

        // Add the other LL Widgets
        SmartDashboard.putNumber("Visible AprilTag TID", -1);
        SmartDashboard.putBoolean("Tracking AprilTag?", false);
        SmartDashboard.putBoolean("MegaTag2?", false);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if(Robot.isSimulation()){

        } else{
            updatePoseEstimation();
        }
    }

    public Command updatePoseEstimationCommand() {
        return runOnce(() -> updatePoseEstimation());
    }

    public Command autoAlignmentCommand(String location) {
        return Commands.defer(() -> autoAlignmentOffset(location), Set.of(swerveSub)); // This is based off of the work
                                                                                       // of teams 910 Foley Freeze and
                                                                                       // 4915 Spartronics. This
                                                                                       // generates a path given the
                                                                                       // bot's current pose and offset
                                                                                       // tag pose
        // return Commands.none();
    }

    public void updatePoseEstimation() {
        // Publish Original Swerve Pose
        currentSwervePose.set(drivebase.getPose());
        autoBuilderPose.set(AutoBuilder.getCurrentPose());

        double tagId = LimelightHelpers.getFiducialID(Constants.LimeLight.LIMELIGHT_NAME);
        LimelightHelpers.SetRobotOrientation(Constants.LimeLight.LIMELIGHT_NAME, drivebase.getYaw().getDegrees(), 0.0,
                0.0, 0.0, 0.0, 0.0);

        try {

            if (LimelightHelpers.getTV(Constants.LimeLight.LIMELIGHT_NAME)) {

                // Signal Tag Visible
                // Led.setColorAlignmentBlink(Color.kLimeGreen);
                Pose2d tagPose = new Pose2d();
                Optional<Pose3d> tagPosePre = Optional.of(Constants.LimeLight.APRILTAG_FIELD_LAYOUT
                        .getTagPose((int) LimelightHelpers.getFiducialID(Constants.LimeLight.LIMELIGHT_NAME)).get());
                if (tagPosePre.isPresent()) {
                    tagPose = tagPosePre.get().toPose2d();

                    if (LimelightHelpers.getTV(Constants.LimeLight.LIMELIGHT_NAME)) {
                        SmartDashboard.putNumber("Visible AprilTag TID", tagId);
                        SmartDashboard.putBoolean("Tracking AprilTag?", true);
                        SmartDashboard.putNumber("Tag Pose X", tagPose.getX());
                        SmartDashboard.putNumber("Tag Pose Y", tagPose.getY());
                    } else {
                        SmartDashboard.putNumber("Visible AprilTag TID", -1); // If no tag, set to an arbitrary -1
                        SmartDashboard.putBoolean("Tracking AprilTag?", false); // If no tag, set the bool widget to red
                                                                                // (false)
                        SmartDashboard.putNumber("Tag Pose X", -1); // If no tag, set to an arbitrary -1
                        SmartDashboard.putNumber("Tag Pose Y", -1); // If no tag, set to an arbitrary -1
                    }

                    LimelightHelpers.PoseEstimate limelightBotPoseEstimateMT2 = LimelightHelpers
                            .getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LimeLight.LIMELIGHT_NAME);
                    LimelightHelpers.PoseEstimate limelightBotPoseEstimateMT = LimelightHelpers
                            .getBotPoseEstimate_wpiBlue(Constants.LimeLight.LIMELIGHT_NAME);

                    LimelightHelpers.PoseEstimate poseToUse = limelightBotPoseEstimateMT;

                    if (fieldBoundary.isPoseWithinArea(poseToUse.pose) && poseToUse.tagCount > 0) { // &&
                                                                                                    // LimelightHelpers.getTX(Constants.LimeLight.LIMELIGHT_NAME)
                                                                                                    // != 0.0){
                        if (limelightBotPoseEstimateMT.avgTagDist < Units.feetToMeters(12)) {
                            poseToUse = limelightBotPoseEstimateMT;
                            SmartDashboard.putBoolean("MegaTag2?", false);
                        } else {
                            poseToUse = limelightBotPoseEstimateMT2;
                            SmartDashboard.putBoolean("MegaTag2?", true);
                        }
                    }
                    Pose2d finalPose = new Pose2d(poseToUse.pose.getX(), poseToUse.pose.getY(),
                            poseToUse.pose.getRotation());
                            //drivebase.getPose().getRotation()); // Ignore Drivebase Numbers
                    //drivebase.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999)); // Standard Deviation !!BROKEN!!
                    //visionPoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999)); // Standard Deviation, !!BROKEN!!
                    drivebase.addVisionMeasurement(finalPose, poseToUse.timestampSeconds); // Add MegaTag Pose, but get the timestamp from the MegaTag2 Pose.
                    
                    // Update Vision Pose Estimator
                    visionPoseEstimator.update(finalPose.getRotation(), drivebase.getModulePositions());
                    visionPoseEstimator.addVisionMeasurement(finalPose, poseToUse.timestampSeconds);

                    // Update Field Widget
                    m_field.setRobotPose(poseToUse.pose);
                    SmartDashboard.putData(m_field);

                    limelightPosePublisher.set(poseToUse.pose);
                    visionPoseEstimatorPublisher.set(visionPoseEstimator.getEstimatedPosition());
                } else {
                    // Led.setColorAlignment(Color.kDarkRed);
                }
                Pose2d drivebaseEstimatedPose = this.drivebase.getPose();
                SmartDashboard.putNumber("Bot Pose Estimation X", drivebaseEstimatedPose.getX()); // Display the
                                                                                                  // estimated bot X
                SmartDashboard.putNumber("Bot Pose Estimation Y", drivebaseEstimatedPose.getY()); // Display the
                                                                                                  // estimated bot Y
            } else {
                SmartDashboard.putNumber("Visible AprilTag TID", -1); // If no tag, set to an arbitrary -1
                SmartDashboard.putBoolean("Tracking AprilTag?", false); // If no tag, set the bool widget to red
                                                                        // (false)
                SmartDashboard.putNumber("Tag Pose X", -1); // If no tag, set to an arbitrary -1
                SmartDashboard.putNumber("Tag Pose Y", -1); // If no tag, set to an arbitrary -1
            }
        } catch (Exception e) {

        }
    }

    public Command autoAlignmentOffset(String location) {

        autoBuilderPose.set(AutoBuilder.getCurrentPose());
        // We don't pathfind UNLESS we can see a tag (For now at least). Otherwise, we
        // could hit an allied bots or opponent defense bots.
        if (LimelightHelpers.getTV(Constants.LimeLight.LIMELIGHT_NAME)) {
            Pose2d tagPose = Constants.LimeLight.APRILTAG_FIELD_LAYOUT
                    .getTagPose((int) LimelightHelpers.getFiducialID(Constants.LimeLight.LIMELIGHT_NAME)).get()
                    .toPose2d(); // Pose for visible tag
            Transform2d offsetTransformation;
            Pose2d startingPose = getCurrentLimelightPose();
            //Pose2d startingPose = drivebase.getPose(); // This is the current pose of the bot // Drivebase Pose is
                                                       // totally broken, bypassing it in favor of direct LL numbers. LL
                                                       // is more accurate
            Pose2d targetPose; // This will be decided below
            if (location.toLowerCase().equalsIgnoreCase("left")) {
                // Left Coral Alignment
                offsetTransformation = new Transform2d(
                        (Constants.LimeLight.ROBOT_SIDE_LENGTH / 2.0) + Constants.LimeLight.BUMPER_WIDTH, // Forward/Backwards
                                                                                                        // Offset
                        Constants.Coral.LEFT_BRANCH_OFFSET, // Horizontal Offset
                        Rotation2d.kZero // Rotation here doesn't matter
                );
                targetPose = invert(tagPose.plus(offsetTransformation)); // Add the offset to the tag's pose and invert
                                                                         // so we face towards the tag, not the
                                                                         // direcetion the tag faces
            } else if (location.toLowerCase().equalsIgnoreCase("center")) {
                // Center/Algae Alignment
                offsetTransformation = new Transform2d(
                        (Constants.LimeLight.ROBOT_SIDE_LENGTH / 2.0) + Constants.LimeLight.BUMPER_WIDTH, // Forward/Backwards
                                                                                                        // Offset
                        Constants.Algae.OFFSET, // Horizontal Offset
                        Rotation2d.kZero // Rotation here doesn't matter
                );
                targetPose = invert(tagPose.plus(offsetTransformation)); // Add the offset to the tag's pose and invert
                                                                         // so we face towards the tag, not the
                                                                         // direcetion the tag faces
            } else if (location.toLowerCase().equalsIgnoreCase("right")) {
                // Right Coral Alignment
                offsetTransformation = new Transform2d(
                        (Constants.LimeLight.ROBOT_SIDE_LENGTH / 2.0) + Constants.LimeLight.BUMPER_WIDTH, // Forward/Backwards
                                                                                                        // Offset
                        Constants.Coral.RIGHT_BRANCH_OFFSET, // Horizontal Offset
                        Rotation2d.kZero // Rotation here doesn't matter
                );
                targetPose = invert(tagPose.plus(offsetTransformation)); // Add the offset to the tag's pose and invert
                                                                         // so we face towards the tag, not the
                                                                         // direcetion the tag faces
            } else {
                targetPose = new Pose2d();
            }

            SmartDashboard.putNumber("Target Pose X", targetPose.getX());
            SmartDashboard.putNumber("Target Pose Y", targetPose.getY());

            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses( // Generate a path given our starting
                                                                           // (current) and target poses. We won't be
                                                                           // far enough away to need much more
                    startingPose,
                    targetPose);

            PathConstraints constraints = new PathConstraints(
                    3.0, // Max Velocity Per Second (Linear) (3 Default)
                    3.0, // Max Acceleration Per Second (Linear) (3 Default)
                    2 * Math.PI, // Max Angular Velocity Per Second (Rotational)
                    4 * Math.PI // Max Angular Acceleration Per Second (Rotational)
            );
            PathPlannerPath path = new PathPlannerPath(
                    waypoints,
                    constraints,
                    new IdealStartingState(
                            averageVelocity(drivebase.getFieldVelocity().vxMetersPerSecond,
                                    drivebase.getFieldVelocity().vyMetersPerSecond),
                            drivebase.getGyroRotation3d().toRotation2d()), // Start with the current velocity and
                                                                           // heading, keeps the transition smoother
                    new GoalEndState(0.0, targetPose.getRotation()));

            path.preventFlipping = true; // If the coords are correct, don't flip it. This keeps us from accidentally
                                         // going to the other side

            // Signal Pathfinding Is Now Controlling Drive
            // Led.setColorAlignmentBlink(Color.kSkyBlue);

            lastPose = targetPose; // This is for transiting between locations.

            startPosePublisher.set(startingPose);
            targetPosePublisher.set(targetPose);

            // return Commands.none();
            return AutoBuilder.followPath(path);
        } else {
            // If we don't see a tag, don't have the free will to pathfind.
            return Commands.none();
        }
    }

    public Command transitToTag(String location, int goalTag) {
        // Transit between scoring locations

        autoBuilderPose.set(AutoBuilder.getCurrentPose());
        // We don't pathfind UNLESS we can see a tag (For now at least). Otherwise, we
        // could hit an allied bots or opponent defense bots.
        //if (LimelightHelpers.getTV(Constants.LimeLight.LIMELIGHT_NAME)) {
            Pose2d tagPose = Constants.LimeLight.APRILTAG_FIELD_LAYOUT
                    .getTagPose((int) goalTag).get()
                    .toPose2d(); // Pose for visible tag
            Transform2d offsetTransformation;
            //Pose2d startingPose = lastPose; // Assum
            Pose2d startingPose = drivebase.getPose(); // This is the current pose of the bot, estimated by the drivebase
            Pose2d targetPose; // This will be decided below
            if (location.toLowerCase().equalsIgnoreCase("left")) {
                // Left Coral Alignment
                offsetTransformation = new Transform2d(
                        (Constants.LimeLight.ROBOT_SIDE_LENGTH / 2.0) + Constants.LimeLight.BUMPER_WIDTH, // Forward/Backwards
                                                                                                        // Offset
                        Constants.Coral.LEFT_BRANCH_OFFSET, // Horizontal Offset
                        Rotation2d.kZero // Rotation here doesn't matter
                );
                targetPose = invert(tagPose.plus(offsetTransformation)); // Add the offset to the tag's pose and invert
                                                                         // so we face towards the tag, not the
                                                                         // direcetion the tag faces
            } else if (location.toLowerCase().equalsIgnoreCase("center")) {
                // Center/Algae Alignment
                offsetTransformation = new Transform2d(
                        (Constants.LimeLight.ROBOT_SIDE_LENGTH / 2.0) + Constants.LimeLight.BUMPER_WIDTH, // Forward/Backwards
                                                                                                        // Offset
                        Constants.Algae.OFFSET, // Horizontal Offset
                        Rotation2d.kZero // Rotation here doesn't matter
                );
                targetPose = invert(tagPose.plus(offsetTransformation)); // Add the offset to the tag's pose and invert
                                                                         // so we face towards the tag, not the
                                                                         // direcetion the tag faces
            } else if (location.toLowerCase().equalsIgnoreCase("right")) {
                // Right Coral Alignment
                offsetTransformation = new Transform2d(
                        (Constants.LimeLight.ROBOT_SIDE_LENGTH / 2.0) + Constants.LimeLight.BUMPER_WIDTH, // Forward/Backwards
                                                                                                        // Offset
                        Constants.Coral.RIGHT_BRANCH_OFFSET, // Horizontal Offset
                        Rotation2d.kZero // Rotation here doesn't matter
                );
                targetPose = invert(tagPose.plus(offsetTransformation)); // Add the offset to the tag's pose and invert
                                                                         // so we face towards the tag, not the
                                                                         // direcetion the tag faces
            } else {
                targetPose = new Pose2d();
            }

            SmartDashboard.putNumber("Target Pose X", targetPose.getX());
            SmartDashboard.putNumber("Target Pose Y", targetPose.getY());

            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses( // Generate a path given our starting
                                                                           // (current) and target poses. We won't be
                                                                           // far enough away to need much more
                    startingPose,
                    targetPose);

            PathConstraints constraints = new PathConstraints(
                    3.0, // Max Velocity Per Second (Linear) (3 Default)
                    3.0, // Max Acceleration Per Second (Linear) (3 Default)
                    2 * Math.PI, // Max Angular Velocity Per Second (Rotational)
                    4 * Math.PI // Max Angular Acceleration Per Second (Rotational)
            );
            PathPlannerPath path = new PathPlannerPath(
                    waypoints,
                    constraints,
                    new IdealStartingState(
                            averageVelocity(drivebase.getFieldVelocity().vxMetersPerSecond,
                                    drivebase.getFieldVelocity().vyMetersPerSecond),
                            drivebase.getGyroRotation3d().toRotation2d()), // Start with the current velocity and
                                                                           // heading, keeps the transition smoother
                    new GoalEndState(0.0, targetPose.getRotation()));

            path.preventFlipping = true; // If the coords are correct, don't flip it. This keeps us from accidentally
                                         // going to the other side

            // Signal Pathfinding Is Now Controlling Drive
            // Led.setColorAlignmentBlink(Color.kSkyBlue);

            startPosePublisher.set(startingPose);
            targetPosePublisher.set(targetPose);

            // return Commands.none();
            return AutoBuilder.followPath(path);
        //} else {
            // If we don't see a tag, don't have the free will to pathfind.
         //   return Commands.none();
        //}
    }

    private static Pose2d invert(Pose2d in) {
        // Inverts Rotation. We want to face the tag, not the direction the tag faces.
        return new Pose2d(in.getTranslation(), in.getRotation().plus(Rotation2d.k180deg)); // Original Inversion
        // return in;
    }

    private static boolean isBlue() {
        // Check if we are Blue Alliance
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    }

    private static double averageVelocity(double vx, double vy) {
        return Math.sqrt(Math.pow(vx, 2) + Math.pow(vy, 2));
    }

    private Pose2d getCurrentLimelightPose() {
        LimelightHelpers.PoseEstimate limelightBotPoseEstimateMT2 = LimelightHelpers
                .getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LimeLight.LIMELIGHT_NAME);
        LimelightHelpers.PoseEstimate limelightBotPoseEstimateMT = LimelightHelpers
                .getBotPoseEstimate_wpiBlue(Constants.LimeLight.LIMELIGHT_NAME);
        LimelightHelpers.PoseEstimate poseToUse = limelightBotPoseEstimateMT;
        if (fieldBoundary.isPoseWithinArea(poseToUse.pose) && poseToUse.tagCount > 0) { // &&
            // LimelightHelpers.getTX(Constants.LimeLight.LIMELIGHT_NAME)
            // != 0.0){
            if (limelightBotPoseEstimateMT.avgTagDist < Units.feetToMeters(12)) {
                poseToUse = limelightBotPoseEstimateMT;
                SmartDashboard.putBoolean("MegaTag2?", false);
            } else {
                poseToUse = limelightBotPoseEstimateMT2;
                SmartDashboard.putBoolean("MegaTag2?", true);
            }
        }
        Pose2d finalPose = new Pose2d(poseToUse.pose.getX(), poseToUse.pose.getY(),
                poseToUse.pose.getRotation());
        return finalPose;
    }

}
