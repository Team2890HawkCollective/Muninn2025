package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// NetworkTables if needed, LimelightHelpers is less pain
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    }

    public void initializeLimelight()
    {

    }

    public void updatePoseEstimation()
    {

    }

}