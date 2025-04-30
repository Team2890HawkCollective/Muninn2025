package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.Spliterator;
import java.util.function.DoubleSupplier;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.ejml.dense.row.linsol.qr.LinearSolverQr_CDRM;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.PathPlannerLogging;

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

    private RobotContainer m_robotContainer; // Reference to the container

    private Pose2d lastPose; // Last Known Pose; Used for tranisting between locations.
    private boolean visionUpdates = false;

    // This is all pushing to Network Tables for display on AdvantageScope.
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

    private final Field2d m_field = new Field2d(); // The Smartdashboard Field Widget

    // The fieldBoundry is gotten from the bottom left, and upper right corners.
    // Bottom Left is always (0,0)
    // Upper Right is the dimensions of the field. I found that this is in meters by
    // comparing the values provided in the example to last year's field dimensions.
    private final RectanglePoseArea fieldBoundary = new RectanglePoseArea(new Translation2d(0, 0),  // (0,0) is BOTTOM LEFT on BLUE! Field is BLUE Origin
            new Translation2d(17.55, 8.05)); // TOP RIGHT on RED

    private final SwerveDrivePoseEstimator visionPoseEstimator; // This is used for the Limelight alone, so it only takes updates from the LL. Required? No. This is for troubleshooting

    public TargetingSubsystem(SwerveSubsystem driveSystem, RobotContainer m_robotContainer) {
        this.m_robotContainer = m_robotContainer;
        this.drivebase = driveSystem.getSwerveDrive(); // Drivetrain OBJECT
        this.swerveSub = driveSystem; // The Drivetrain CLASS
        LimelightHelpers.SetFiducialIDFiltersOverride(Constants.LimeLight.LIMELIGHT_NAME, Constants.LimeLight.ALL_REEF_APRILTAGS); // Filter Out Non-Reef tags

        this.visionPoseEstimator = new SwerveDrivePoseEstimator( // Semi-Indepentant Pose Estimator. It primarily recieves updates from the LL, and minimal info from the drive. Used for debugging.
                drivebase.kinematics,
                drivebase.getGyro().getRotation3d().toRotation2d(),  // Gyro Rotation
                new SwerveModulePosition[] {
                        drivebase.getModulePositions()[0], // Front Left
                        drivebase.getModulePositions()[0], // Front Right
                        drivebase.getModulePositions()[0], // Back Left
                        drivebase.getModulePositions()[0] // Back Right
                },
                Pose2d.kZero,
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)), // In the example, idk what it does
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))); // ^
        
        lastPose = drivebase.getPose();  // Set Last Known Pose To Current Pose At Intialization (Startup)

        SmartDashboard.putData("Field", m_field); // Add the field widget (Not working rn)

        SmartDashboard.putNumber("Target Pose X", -1); // Add Target Pose X display. set to arbitrary -1.
        SmartDashboard.putNumber("Target Pose Y", -1); // Add Target Pose Y display. set to arbitrary -1.
        SmartDashboard.putNumber("Tag Pose X", -1); // Add the Tag Pose X display, set to arbitrary -1.
        SmartDashboard.putNumber("Tag Pose Y", -1); // Add the Tag Pose Y display, set to arbitrary -1.

        // Add the other LL Widgets
        // Also resets the widgets to remove residual info
        SmartDashboard.putNumber("Visible AprilTag TID", -1); // Default to -1. -1 is used because no pose can be negative.
        SmartDashboard.putBoolean("Tracking AprilTag?", false); // Is a allowed (not filtered out) tag visible? Default to false.
        SmartDashboard.putBoolean("MegaTag2?", false); // Are we using MegaTag or MegaTag2? Default to false.
        SmartDashboard.putBoolean("Vision Updates?", false); // Indicator for when vision is/isnt enabled
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if(Robot.isSimulation()){ // Don't run vision in the Sim, it doesn't work.

        } else{
            updatePoseEstimation();
        }
    }

    public void enableVisionUpdates(){
        visionUpdates = true;
        SmartDashboard.putBoolean("Vision Updates?", true);
    }

    public void disableVisionUpdates(){
        visionUpdates = false;
        SmartDashboard.putBoolean("Vision Updates?", false);
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
        currentSwervePose.set(drivebase.getPose()); // Update the debugging publishers
        autoBuilderPose.set(AutoBuilder.getCurrentPose()); // ^

        SmartDashboard.putBoolean("Vision Updates?", visionUpdates); // Update the vision enable/disable indicator. Just a double check to ensure updates.

        double tagId = LimelightHelpers.getFiducialID(Constants.LimeLight.LIMELIGHT_NAME); // Get the ID of the visible tag. NOTE: It returns a DOUBLE for some odd reason.
        LimelightHelpers.SetRobotOrientation(Constants.LimeLight.LIMELIGHT_NAME, drivebase.getYaw().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0); // Updates the angle the LL is facing. Must be called often and BEFORE getting numbers from the LL.

        try {

            if (visionUpdates && LimelightHelpers.getTV(Constants.LimeLight.LIMELIGHT_NAME)) { // If VISION UPDATES are enabled (true) and a tag is visible.

                Led.setColorAlignmentBlink(Color.kLawnGreen); // Signal Tag Visible with the LEDs
                Pose2d tagPose = new Pose2d(); // Empty Pose, will fill with the pose of the tag we see.
                Optional<Pose3d> tagPosePre = Optional.of(Constants.LimeLight.APRILTAG_FIELD_LAYOUT
                        .getTagPose((int) LimelightHelpers.getFiducialID(Constants.LimeLight.LIMELIGHT_NAME)).get()); // This returns a weird type, and a 3d Pose.
                if (tagPosePre.isPresent()) { // If the Optional<> has an actual value.
                    tagPose = tagPosePre.get().toPose2d(); // Get the pose and convert to a 2D Pose

                    if (LimelightHelpers.getTV(Constants.LimeLight.LIMELIGHT_NAME)) { // Double check that we still see a tag
                        SmartDashboard.putNumber("Visible AprilTag TID", tagId); // Put the Tag ID on the dashboard
                        SmartDashboard.putBoolean("Tracking AprilTag?", true); // Turn this dashboard widget green (true)
                        SmartDashboard.putNumber("Tag Pose X", tagPose.getX()); // Put the Tag's pose on the dashboard. (To see if the LL is reading properly)
                        SmartDashboard.putNumber("Tag Pose Y", tagPose.getY()); // ^
                    } else {
                        SmartDashboard.putNumber("Visible AprilTag TID", -1); // If no tag, set to an arbitrary -1
                        SmartDashboard.putBoolean("Tracking AprilTag?", false); // If no tag, set the bool widget to red (false)
                        SmartDashboard.putNumber("Tag Pose X", -1); // If no tag, set to an arbitrary -1
                        SmartDashboard.putNumber("Tag Pose Y", -1); // If no tag, set to an arbitrary -1
                    }

                    LimelightHelpers.PoseEstimate limelightBotPoseEstimateMT2 = LimelightHelpers // MegaTag2 Pose (Has been about 90 Degrees off of the actual pose)
                            .getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LimeLight.LIMELIGHT_NAME);
                    LimelightHelpers.PoseEstimate limelightBotPoseEstimateMT = LimelightHelpers // MegaTag Pose (More reliable for us)
                            .getBotPoseEstimate_wpiBlue(Constants.LimeLight.LIMELIGHT_NAME);

                    LimelightHelpers.PoseEstimate poseToUse = limelightBotPoseEstimateMT; // Default to the more reliable reading (MegaTag)

                    if (fieldBoundary.isPoseWithinArea(poseToUse.pose) && poseToUse.tagCount > 0) { // If we are inside the field, this is a wee bit important
                        if (limelightBotPoseEstimateMT.avgTagDist < Units.feetToMeters(12)) { // If we are close (typically are) use the more reliable MegaTag Pose
                            poseToUse = limelightBotPoseEstimateMT;
                            SmartDashboard.putBoolean("MegaTag2?", false); // Not using MT2, so make the box false
                        } else { // If we are far, use MT2. It is supposed to be more reliable at distance, we haven't tested this
                            poseToUse = limelightBotPoseEstimateMT2;
                            SmartDashboard.putBoolean("MegaTag2?", true); // Using MT2, make the box green.
                        }
                    }
                    Pose2d finalPose = new Pose2d(poseToUse.pose.getX(), poseToUse.pose.getY(), poseToUse.pose.getRotation()); // Create a final pose from the used pose
                    drivebase.addVisionMeasurement(finalPose, poseToUse.timestampSeconds); // Add MegaTag Pose, but get the timestamp from the MegaTag2 Pose.
                    // NOTE: DO NOT use the stdevs. This is what was killing us early on. LL's example (which uses stdevs) is WRONG
                    
                    // Update Vision Pose Estimator
                    visionPoseEstimator.update(finalPose.getRotation(), drivebase.getModulePositions()); // Updated the vision estimator with the final pose ROTATION and the drive modules.
                    visionPoseEstimator.addVisionMeasurement(finalPose, poseToUse.timestampSeconds); // Add the final pose to the vision estimator

                    // Update Field Widget
                    m_field.setRobotPose(poseToUse.pose); // Set the robot's current pose on the Field widget to the LL estimated pose. Probably better to use the drive's estimated pose AFTER adding the LL vision measurement
                    SmartDashboard.putData(m_field); // Send the data to the dashboard

                    limelightPosePublisher.set(poseToUse.pose);                                     // Update the debugging publisher 
                    visionPoseEstimatorPublisher.set(visionPoseEstimator.getEstimatedPosition());   // ^
                } else {
                        Led.setColorAlignment(Color.kWhite); // No tag visible, so set the lights to white
                }
                Pose2d drivebaseEstimatedPose = this.drivebase.getPose(); // Get the drive's estimated pose.
                SmartDashboard.putNumber("Bot Pose Estimation X", drivebaseEstimatedPose.getX()); // Display the estimated bot X
                SmartDashboard.putNumber("Bot Pose Estimation Y", drivebaseEstimatedPose.getY()); // Display the estimated bot Y
            } else {
                Led.setColorAlignment(Color.kWhite); // No tag visible, set the lights to white.
                SmartDashboard.putNumber("Visible AprilTag TID", -1); // If no tag, set to an arbitrary -1
                SmartDashboard.putBoolean("Tracking AprilTag?", false); // If no tag, set the bool widget to red (false)
                SmartDashboard.putNumber("Tag Pose X", -1); // If no tag, set to an arbitrary -1
                SmartDashboard.putNumber("Tag Pose Y", -1); // If no tag, set to an arbitrary -1
            }
        } catch (Exception e) {
            // Do nothing if error. The try/catch isn't needed, we just added it when trying to figure out the Optional<> thing.
        }
    }

    public Command autoAlignmentOffset(String location) { // Alignment based on pose offset.

        autoBuilderPose.set(AutoBuilder.getCurrentPose()); // Update a debugging publisher with PathPlanner's current pose. NOTE: This is based on where drive thinks it is.
        // We don't pathfind UNLESS we can see a tag (For now at least). Otherwise, we
        // could hit an allied bots or opponent defense bots.
        if (LimelightHelpers.getTV(Constants.LimeLight.LIMELIGHT_NAME)) { // If a tag is visible
            Pose2d tagPose = Constants.LimeLight.APRILTAG_FIELD_LAYOUT // Get the April Tag Field Layout that is loaded in Constants.
                    .getTagPose((int) LimelightHelpers.getFiducialID(Constants.LimeLight.LIMELIGHT_NAME)).get() // Get the visble tag ID
                    .toPose2d(); // Pose for visible tag converted from 3d to 2d
            Transform2d offsetTransformation;
            Pose2d startingPose = getCurrentLimelightPose(); // Get the start pose from the LL exclusively
            //Pose2d startingPose = drivebase.getPose(); // This is the current pose of the bot based on where the drivetrain thinks it is. You can use this or the line above.
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

            SmartDashboard.putNumber("Target Pose X", targetPose.getX()); // Send the tag's pose to the dashboard. Used to verify that we aren't getting crazy readings
            SmartDashboard.putNumber("Target Pose Y", targetPose.getY()); // ^

            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses( // Generate a path given our starting
                                                                           // (current) and target poses. We won't be
                                                                           // far enough away to need much more
                    startingPose,  // Start Pose
                    targetPose);   // End Pose

            PathConstraints constraints = new PathConstraints(
                    5.0, // Max Velocity Per Second (Linear) (3 Default)
                    5.0, // Max Acceleration Per Second (Linear) (3 Default)
                    2 * Math.PI, // Max Angular Velocity Per Second (Rotational)
                    4 * Math.PI // Max Angular Acceleration Per Second (Rotational)
            );
            PathPlannerPath path = new PathPlannerPath(  // Create a PathPlanner path
                    waypoints, // The waypoint list created above
                    constraints, // The constraints set above
                    new IdealStartingState( // This is the expected starting state of the robot when PathPlanner takes over
                            averageVelocity(drivebase.getFieldVelocity().vxMetersPerSecond, drivebase.getFieldVelocity().vyMetersPerSecond), // Do physics math in order to get the full velocity based on the X&Y vectors
                            drivebase.getGyroRotation3d().toRotation2d()), // Start with the current velocity and heading, keeps the transition smoother
                    new GoalEndState(0.0, targetPose.getRotation())); // Where we want to end. Anything other than 0.0 as the velocity arg means the bot WONT stop!!!

            path.preventFlipping = true; // If the coords are correct, don't flip it. This keeps us from accidentally
                                         // going to the other side

            // Signal Pathfinding Is Now Controlling Drive
            Led.setColorAlignmentBlink(Color.kSkyBlue);

            //Post Path To Field2d Widget!!
            PathPlannerLogging.setLogActivePathCallback((poses)->{
                m_field.getObject("path").setPoses(poses);
            });

            lastPose = targetPose; // This is for transiting between locations.

            startPosePublisher.set(startingPose); // Update a debugging publisher
            targetPosePublisher.set(targetPose); // ^

            disableVisionUpdates(); // Disable vision during pathfinding, so as to not interfere with PathPlanner

            // return Commands.none();
            return AutoBuilder.followPath(path); // Return the PathPlanner command
        } else {
            // If we don't see a tag, don't have the free will to pathfind.
            return Commands.none();
        }
    }

    public Command transitToTag(String location, int goalTag) { // Modifed autoAligmentOffset code. This is for transiting to other positions. NOTE: This is set so that it WILL run without seeing a tag. Instead, it goes to a target given a ending tag. Can be used with ANY tag.
        // Transit between scoring locations

        autoBuilderPose.set(AutoBuilder.getCurrentPose());
        // This will run even if a tag isn't visible
            Pose2d tagPose = Constants.LimeLight.APRILTAG_FIELD_LAYOUT
                    .getTagPose((int) goalTag).get()
                    .toPose2d(); // Pose for visible tag
            Transform2d offsetTransformation;
            Pose2d startingPose = drivebase.getPose(); // This is the current pose of the bot, estimated by the drivebase. Must use this because a tag isn't going to be visible if we are moving from a scoreing postion
            Pose2d targetPose; // This will be decided below
            //NOTE: Add in more IFs. There are slightly different Transformation constants needed if we are going to a coral staton, barge, or somewhere else.
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

            SmartDashboard.putNumber("Target Pose X", targetPose.getX()); // Send the tag's pose to the dashboard. Used to verify that we aren't getting crazy readings
            SmartDashboard.putNumber("Target Pose Y", targetPose.getY()); // ^

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
        return Math.sqrt(Math.pow(vx, 2) + Math.pow(vy, 2)); // Physivs math for getting the true velocity provided the X&Y vectors.
    }

    private Pose2d getCurrentLimelightPose() { // Get the pose purely from the LL. This is just a partial copy of what's in the updatePoseEstimation method.
        LimelightHelpers.PoseEstimate limelightBotPoseEstimateMT2 = LimelightHelpers
                .getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LimeLight.LIMELIGHT_NAME);
        LimelightHelpers.PoseEstimate limelightBotPoseEstimateMT = LimelightHelpers
                .getBotPoseEstimate_wpiBlue(Constants.LimeLight.LIMELIGHT_NAME);
        LimelightHelpers.PoseEstimate poseToUse = limelightBotPoseEstimateMT;
        if (fieldBoundary.isPoseWithinArea(poseToUse.pose) && poseToUse.tagCount > 0) {
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

    public Command stringRunAuton(){
        String autonCode = m_robotContainer.m_shuffleboardDisplay.getAutonInputCode();
        String[] autonCodes = autonCode.split("/");
        DriverStation.reportError(autonCode, false);
        for(String code : autonCodes){
            String[] splitList = splitCode(code);
            String location = "none";
            if(splitList[2].equalsIgnoreCase("r")){
                location = "left";
            }
            if(splitList[2].equalsIgnoreCase("r")){
                location = "right";
            }
            if(splitList[2].equalsIgnoreCase("c")){
                location = "center";
            }
            Command transitCommand = transitToTag(location, Integer.valueOf(splitList[0]));
            Command elevatorCommand = m_robotContainer.m_ElevatorSubsystem.goToElevatorStageCommand(Integer.valueOf(splitList[3]));
            Command manipulatorCommand = Commands.none();
            if(splitList[1].equalsIgnoreCase("r")){ // R/r == REEF
                if(Integer.valueOf(splitList[3]) >= 1 && Integer.valueOf(splitList[3]) <= 4){
                    manipulatorCommand = m_robotContainer.m_CoralSubsystem.coralFullCommand(Integer.valueOf(splitList[3]));
                } else if(Integer.valueOf(splitList[3]) >= 5 && Integer.valueOf(splitList[3]) <= 6){
                    manipulatorCommand = m_robotContainer.m_AlgaeSubsystem.algaeFullCommand();
                } else {
                    manipulatorCommand = Commands.none();
                }
            }
            Command fullCommand = Commands.sequence(transitCommand, elevatorCommand, manipulatorCommand);
            CommandScheduler.getInstance().schedule(fullCommand);
        }
        return Commands.none();
    }

    private static String[] splitCode(String code){
        String[] splitList = new String[4];
        String regex = "(\\d{2})([A-Za-z])([A-Za-z])(\\d)";
        Pattern pattern = Pattern.compile(regex);
        Matcher matcher = pattern.matcher(code);
        if(matcher.matches()){
            splitList[0] = matcher.group(1); // April Tag (2 Digits!!! Ex. 06, 17, etc...)
            splitList[1] = matcher.group(2); // Goal (Reef (R), Coral Station (C), Barge (B), Processor (P))
            splitList[2] = matcher.group(3); // Reef ONLY!! L = Left, R = Right, C = Center
            splitList[3] = matcher.group(4); // Reef ONLY!! Level: 1-4 are Coral, 5-6 are Algae, -1 is NO movement, 7 is Algae in Barge
        }
        return splitList;
    }
}

