package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.commands.AlignToReefTagRelative;
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

    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
        .getStructTopic("MyPose", Pose2d.struct).publish();
    StructArrayPublisher<Pose2d> arrayPublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("MyPoseArray", Pose2d.struct).publish();

    private final Field2d m_field = new Field2d();

    // The fieldBoundry is gotten from the bottom left, and upper right corners.
    // Bottom Left is always (0,0)
    // Upper Right is the dimensions of the field. I found that this is in meters by
    // comparing the values provided in the example to last year's field dimensions.
    private final RectanglePoseArea fieldBoundary = new RectanglePoseArea(new Translation2d(0, 0),
            new Translation2d(17.55, 8.05));

    private final SwerveDrivePoseEstimator m_poseEstimator;

    public TargetingSubsystem(SwerveSubsystem driveSystem) {
        this.drivebase = driveSystem.getSwerveDrive();
        this.swerveSub = driveSystem;
        LimelightHelpers.SetFiducialIDFiltersOverride(Constants.LimeLight.LIMELIGHT_NAME,
                Constants.LimeLight.ALL_REEF_APRILTAGS); // Filter Out Non-Reef tags

        this.m_poseEstimator = new SwerveDrivePoseEstimator(
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
        // updatePoseEstimation();
    }

    public Command updatePoseEstimationCommand() {
        return runOnce(() -> updatePoseEstimation());
    }

    public Command autoAlignmentCommand(String location) {
        // return Commands.defer(()->autoAlignmentPose(location),Set.of(swerveSub)); // This is the pathfindToPose version.
        //return Commands.defer(()->autoAlignmentOffset(location),Set.of(swerveSub)); // This is based off of the work of teams 910 Foley Freeze and 4915 Spartronics. This generates a path given the bot's current pose and offset tag pose
        // return Commands.defer(()->autoAlignmentBasic(),Set.of(swerveSub));
        return Commands.defer(()->alignToReefTagRelativeCommand(location),Set.of(swerveSub));
        // return Commands.none();
    }

    public Command alignToReefTagRelativeCommand(String location){
        if(location.toLowerCase().equals("left")){
            AlignToReefTagRelative align = new AlignToReefTagRelative(false,swerveSub);
            return align;
        } else if(location.toLowerCase().equals("right")){
            AlignToReefTagRelative align = new AlignToReefTagRelative(true,swerveSub);
            return align;
        } else {
            return Commands.none();
        }
    }
    
    public Command pathfindTest() {
        PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(
                Constants.LimeLight.RedReefPositions.CoralPoses.A,
                constraints,
                0.0 // Goal end velocity in meters/sec
        ).andThen(AutoBuilder.pathfindToPose(
                Constants.LimeLight.RedReefPositions.CoralPoses.L,
                constraints,
                0.0 // Goal end velocity in meters/sec
        ));
    }

    public void initializeLimeLight() {
        LimelightHelpers.SetFiducialIDFiltersOverride(Constants.LimeLight.LIMELIGHT_NAME,
                Constants.LimeLight.ALL_REEF_APRILTAGS); // Filter Out Non-Reef tags
        // Set initial bot orientation
        // Params: Limelight Name, Yaw, Yaw Rate, Pitch, Pitch Rate, Roll, Roll Rate
        LimelightHelpers.SetRobotOrientation(Constants.LimeLight.LIMELIGHT_NAME, drivebase.getYaw().getDegrees(), 0,
                drivebase.getPitch().getDegrees(), 0, drivebase.getRoll().getDegrees(), 0);
    }

    public void updatePoseEstimation() {
        double tagId = LimelightHelpers.getFiducialID(Constants.LimeLight.LIMELIGHT_NAME);
        LimelightHelpers.SetRobotOrientation(Constants.LimeLight.LIMELIGHT_NAME, drivebase.getYaw().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0);

        if (LimelightHelpers.getTV(Constants.LimeLight.LIMELIGHT_NAME)) {

            // Signal Tag Visible
            //Led.setColorAlignmentBlink(Color.kLimeGreen);

            Pose2d tagPose = new Pose2d();
            Optional<Pose3d> tagPosePre = Optional.ofNullable(Constants.LimeLight.APRILTAG_FIELD_LAYOUT
                    .getTagPose((int) LimelightHelpers.getFiducialID(Constants.LimeLight.LIMELIGHT_NAME)).get());
            if(tagPosePre != null){
                tagPose = tagPosePre.get().toPose2d();

                if (LimelightHelpers.getTV(Constants.LimeLight.LIMELIGHT_NAME)) {
                    SmartDashboard.putNumber("Visible AprilTag TID", tagId);
                    SmartDashboard.putBoolean("Tracking AprilTag?", true);
                    SmartDashboard.putNumber("Tag Pose X", tagPose.getX());
                    SmartDashboard.putNumber("Tag Pose Y", tagPose.getY());
                } else {
                    SmartDashboard.putNumber("Visible AprilTag TID", -1); // If no tag, set to an arbitrary -1
                    SmartDashboard.putBoolean("Tracking AprilTag?", false); // If no tag, set the bool widget to red (false)
                    SmartDashboard.putNumber("Tag Pose X", -1); // If no tag, set to an arbitrary -1
                    SmartDashboard.putNumber("Tag Pose Y", -1); // If no tag, set to an arbitrary -1
                }

                LimelightHelpers.PoseEstimate limelightBotPoseEstimateMT2 = LimelightHelpers
                        .getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LimeLight.LIMELIGHT_NAME);
                LimelightHelpers.PoseEstimate limelightBotPoseEstimateMT = LimelightHelpers
                        .getBotPoseEstimate_wpiBlue(Constants.LimeLight.LIMELIGHT_NAME);
                // LimelightHelpers.LimelightTarget_Fiducial jsonData = new
                // LimelightHelpers.LimelightTarget_Fiducial();
                // LimelightHelpers.LimelightTarget_Fiducial jsonData = new
                // LimelightHelpers.LimelightResults.geLatestResults(Constants.Limelight.LIMELIGHT_NAME).targets_Fiducials[0];
                // // It should be one of these two calls
                // Pose2d estimatedFieldPose = jsonData.getRobotPose_TargetSpace2D();
                // m_field.setRobotPose(estimatedFieldPose);

                LimelightHelpers.PoseEstimate poseToUse = limelightBotPoseEstimateMT;

                // Pose2d drivebaseEstimatedPose = this.drivebase.getPose();

                // SmartDashboard.putNumber("Limelight Bot Pose Estimation X",
                // limelightBotPoseEstimateMT2.pose.getX());
                // SmartDashboard.putNumber("Limelight Bot Pose Estimation Y",
                // limelightBotPoseEstimateMT2.pose.getY());
                // SmartDashboard.putNumber("Limelight Target Pose Estimation X",
                // LimelightHelpers.getTX(Constants.LimeLight.LIMELIGHT_NAME));
                // SmartDashboard.putNumber("Limelight Target Pose Estimation Y",
                // LimelightHelpers.getTY(Constants.LimeLight.LIMELIGHT_NAME));
                // SmartDashboard.putNumber("Limelight Bot Pose (Field Space) Estimation X",
                // estimatedFieldPose.getX());
                // SmartDashboard.putNumber("Limelight Bot Pose (Field Space) Estimation Y",
                // estimatedFieldPose.getY());

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
                        drivebase.getPose().getRotation());
                drivebase.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999)); // Standard Deviation
                m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999)); // Standard Deviation
                drivebase.addVisionMeasurement(finalPose, poseToUse.timestampSeconds); // Add Field Pose, but get the
                                                                                    // timestamp from the MegaTag2 Pose.
                m_poseEstimator.addVisionMeasurement(finalPose, poseToUse.timestampSeconds);
                // drivebase.addVisionMeasurement(limelightBotPoseEstimateMT2.pose,
                // limelightBotPoseEstimateMT2.timestampSeconds);
                m_field.setRobotPose(poseToUse.pose);
                SmartDashboard.putData(m_field);
                
                arrayPublisher.set(new Pose2d[] {drivebase.getPose(), finalPose});
            } else {
                //Led.setColorAlignment(Color.kDarkRed);
            }
            Pose2d drivebaseEstimatedPose = this.drivebase.getPose();
            SmartDashboard.putNumber("Bot Pose Estimation X", drivebaseEstimatedPose.getX()); // Display the estimated bot X
            SmartDashboard.putNumber("Bot Pose Estimation Y", drivebaseEstimatedPose.getY()); // Display the estimated bot Y
        }
    }

    public Command autoAlignmentPose(String location) {
        PathConstraints constraints = new PathConstraints(
                0.05, 0.07, // Default MaxVelocity: 3.0; Max Acceleration: 4.0
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        int tagId = (int) LimelightHelpers.getFiducialID(Constants.LimeLight.LIMELIGHT_NAME);

        Pose2d targetPose = new Pose2d();

        if (LimelightHelpers.getTV(Constants.LimeLight.LIMELIGHT_NAME)) {
            // Red Reef Tags
            if (tagId == 7) {
                if(location.toLowerCase().equals("left")) {
                    targetPose = Constants.LimeLight.RedReefPositions.CoralPoses.A;
                } else if(location.toLowerCase().equals("center")) {
                    targetPose = Constants.LimeLight.RedReefPositions.AlgaePoses.THREE;
                } else if(location.toLowerCase().equals("right")) {
                    targetPose = Constants.LimeLight.RedReefPositions.CoralPoses.B;
                }
            }
            if (tagId == 8) {
                if(location.toLowerCase().equals("left")) {
                    targetPose = Constants.LimeLight.RedReefPositions.CoralPoses.C;
                } else if(location.toLowerCase().equals("center")) {
                    targetPose = Constants.LimeLight.RedReefPositions.AlgaePoses.ONE;
                } else if(location.toLowerCase().equals("right")) {
                    targetPose = Constants.LimeLight.RedReefPositions.CoralPoses.D;
                }
            }
            if (tagId == 9) {
                if(location.toLowerCase().equals("left")) {
                    targetPose = Constants.LimeLight.RedReefPositions.CoralPoses.E;
                } else if(location.toLowerCase().equals("center")) {
                    targetPose = Constants.LimeLight.RedReefPositions.AlgaePoses.ELEVEN;
                } else if(location.toLowerCase().equals("right")) {
                    targetPose = Constants.LimeLight.RedReefPositions.CoralPoses.F;
                }
            }
            if (tagId == 10) {
                if(location.toLowerCase().equals("left")) {
                    targetPose = Constants.LimeLight.RedReefPositions.CoralPoses.G;
                } else if(location.toLowerCase().equals("cneter")) {
                    targetPose = Constants.LimeLight.RedReefPositions.AlgaePoses.NINE;
                } else if(location.toLowerCase().equals("right")) {
                    targetPose = Constants.LimeLight.RedReefPositions.CoralPoses.H;
                }
            }
            if (tagId == 11) {
                if(location.toLowerCase().equals("left")) {
                    targetPose = Constants.LimeLight.RedReefPositions.CoralPoses.I;
                }else if(location.toLowerCase().equals("center")) {
                    targetPose = Constants.LimeLight.RedReefPositions.AlgaePoses.SEVEN;
                } else if(location.toLowerCase().equals("right")) {
                    targetPose = Constants.LimeLight.RedReefPositions.CoralPoses.J;
                }
            }
            if (tagId == 6) {
                if(location.toLowerCase().equals("left")) {
                    targetPose = Constants.LimeLight.RedReefPositions.CoralPoses.K;
                } else if(location.toLowerCase().equals("center")) {
                    targetPose = Constants.LimeLight.RedReefPositions.AlgaePoses.FIVE;
                }else if(location.toLowerCase().equals("right")) {
                    targetPose = Constants.LimeLight.RedReefPositions.CoralPoses.L;
                }
            }

            // Blue Reef Tags
            if (tagId == 18) {
                if(location.toLowerCase().equals("left")) {
                    targetPose = Constants.LimeLight.BlueReefPositions.CoralPoses.A;
                }else if(location.toLowerCase().equals("center")) {
                    targetPose = Constants.LimeLight.BlueReefPositions.AlgaePoses.THREE;
                } else if(location.toLowerCase().equals("left")) {
                    targetPose = Constants.LimeLight.BlueReefPositions.CoralPoses.B;
                }
            }
            if (tagId == 17) {
                if(location.toLowerCase().equals("left")) {
                    targetPose = Constants.LimeLight.BlueReefPositions.CoralPoses.C;
                } else if(location.toLowerCase().equals("center")) {
                    targetPose = Constants.LimeLight.BlueReefPositions.AlgaePoses.ONE;
                } else if(location.toLowerCase().equals("right")) {
                    targetPose = Constants.LimeLight.BlueReefPositions.CoralPoses.D;
                }
            }
            if (tagId == 22) {
                if(location.toLowerCase().equals("left")) {
                    targetPose = Constants.LimeLight.BlueReefPositions.CoralPoses.E;
                } else if(location.toLowerCase().equals("center")) {
                    targetPose = Constants.LimeLight.BlueReefPositions.AlgaePoses.ELEVEN;
                } else if(location.toLowerCase().equals("right")) {
                    targetPose = Constants.LimeLight.BlueReefPositions.CoralPoses.F;
                }
            }
            if (tagId == 21) {
                if(location.toLowerCase().equals("left")) {
                    targetPose = Constants.LimeLight.BlueReefPositions.CoralPoses.G;
                } else if(location.toLowerCase().equals("center")) {
                    targetPose = Constants.LimeLight.BlueReefPositions.AlgaePoses.NINE;
                } else if(location.toLowerCase().equals("right")) {
                    targetPose = Constants.LimeLight.BlueReefPositions.CoralPoses.H;
                }
            }
            if (tagId == 20) {
                if(location.toLowerCase().equals("left")) {
                    targetPose = Constants.LimeLight.BlueReefPositions.CoralPoses.I;
                } else if(location.toLowerCase().equals("center")) {
                    targetPose = Constants.LimeLight.BlueReefPositions.AlgaePoses.SEVEN;
                } else if(location.toLowerCase().equals("right")) {
                    targetPose = Constants.LimeLight.BlueReefPositions.CoralPoses.J;
                }
            }
            if (tagId == 19) {
                if(location.toLowerCase().equals("left")) {
                    targetPose = Constants.LimeLight.BlueReefPositions.CoralPoses.K;
                } else if(location.toLowerCase().equals("center")) {
                    targetPose = Constants.LimeLight.BlueReefPositions.AlgaePoses.FIVE;
                } else if(location.toLowerCase().equals("right")) {
                    targetPose = Constants.LimeLight.BlueReefPositions.CoralPoses.L;
                }
            }
            return AutoBuilder.pathfindToPose(
                    targetPose,
                    constraints,
                    0.0 // Goal end velocity in meters/sec
            );
        } else {
            return Commands.none();
        }

    }

    public Command autoAlignmentOffset(String location) {
        // We don't pathfind UNLESS we can see a tag (For now at least). Otherwise, we
        // could hit an allied bots or opponent defense bots.
        if (LimelightHelpers.getTV(Constants.LimeLight.LIMELIGHT_NAME)) {
            Pose2d tagPose = Constants.LimeLight.APRILTAG_FIELD_LAYOUT
                    .getTagPose((int) LimelightHelpers.getFiducialID(Constants.LimeLight.LIMELIGHT_NAME)).get()
                    .toPose2d(); // Pose for visible tag
            Transform2d offsetTransformation;
            Pose2d startingPose = drivebase.getPose(); // This is the current pose of the bot
            Pose2d targetPose; // This will be decided below
            if (location.toLowerCase().equalsIgnoreCase("left")) {
                // Left Coral Alignment
                offsetTransformation = new Transform2d(
                        Constants.LimeLight.ROBOT_SIDE_LENGTH / 2.0 + Constants.LimeLight.BUMPER_WIDTH, // Forward/Backwards
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
                        Constants.LimeLight.ROBOT_SIDE_LENGTH / 2.0 + Constants.LimeLight.BUMPER_WIDTH, // Forward/Backwards
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
                        Constants.LimeLight.ROBOT_SIDE_LENGTH / 2.0 + Constants.LimeLight.BUMPER_WIDTH, // Forward/Backwards
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
                    3.0, // Max Velocity Per Second (Linear)
                    3.0, // Max Acceleration Per Second (Linear)
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
            //Led.setColorAlignmentBlink(Color.kSkyBlue);

            publisher.set(targetPose);

            // return Commands.none();
            return AutoBuilder.followPath(path);
        } else {
            // If we don't see a tag, don't have the free will to pathfind.
            return Commands.none();
        }
    }

    public Command autoAlignmentBasic() {
        DoubleSupplier xSpeed = ()->limelight_range_proportional();
        DoubleSupplier ySpeed = ()->0;
        DoubleSupplier rot = ()->limelight_aim_proportional();
        return swerveSub.driveCommand(xSpeed,ySpeed, rot, false);
    }

    double limelight_aim_proportional() {
        // kP (constant of proportionality)
        // this is a hand-tuned number that determines the aggressiveness of our
        // proportional control loop
        // if it is too high, the robot will oscillate around.
        // if it is too low, the robot will never reach its target
        // if the robot never turns in the correct direction, kP should be inverted.
        double kP = .035;

        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the
        // rightmost edge of
        // your limelight 3 feed, tx should return roughly 31 degrees.
        double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

        // convert to radians per second for our drive method
        targetingAngularVelocity *= swerveSub.getSwerveDrive().getMaximumChassisAngularVelocity();

        // invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= -1.0;

        return targetingAngularVelocity;
    }

    // simple proportional ranging control with Limelight's "ty" value
    // this works best if your Limelight's mount height and target mount height are
    // different.
    // if your limelight and target are mounted at the same or similar heights, use
    // "ta" (area) for target ranging rather than "ty"
    double limelight_range_proportional() {
        double kP = .1;
        double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
        targetingForwardSpeed *= swerveSub.getSwerveDrive().getMaximumChassisVelocity();
        targetingForwardSpeed *= -1.0;
        return targetingForwardSpeed;
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

    private static Pose2d invert(Pose2d in) {
        // Inverts Rotation. We want to face the tag, not the direction the tag faces.
        return new Pose2d(in.getTranslation(), in.getRotation().plus(Rotation2d.k180deg)); // Original Inversion
    }

    private static boolean isBlue() {
        // Check if we are Blue Alliance
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    }

    private static double averageVelocity(double vx, double vy) {
        return Math.sqrt(Math.pow(vx, 2) + Math.pow(vy, 2));
    }


    
}
