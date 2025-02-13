package frc.robot;
import frc.robot.Constants;
import frc.robot.Constants.ShuffleboardConstants;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.motors.SwerveMotor;
import swervelib.parser.SwerveModuleConfiguration;

import java.util.Map;

import javax.lang.model.type.NullType;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; //Some things have to go through SmartDashboard
import edu.wpi.first.wpilibj2.command.Command;

public class ShuffleboardDisplay {

    private ShuffleboardTab mainTab = Shuffleboard.getTab("Main");
    //private ShuffleboardTab PID_Tab = Shuffleboard.getTab("PID_Tab");
    private final SendableChooser<String> universalModeChooser = new SendableChooser<>();
    private SendableChooser<Command> autoChooser;
    private GenericEntry genericEntryTest = mainTab.add("Generic",0).getEntry();


    public void initiateDisplay(){
        universalModeChooser.setDefaultOption("Competition Autos","competiton");
        universalModeChooser.addOption("Testing: Test Autos", "testing");
        universalModeChooser.addOption("Testing: All Autos", "allAutos");
        SmartDashboard.putData("Universal Mode Chooser", universalModeChooser);
        genericEntryTest.setDouble(12.3);

        //SmartDashboard.putNumber("Test",1);
    }

    public void initializeAutoChooser(){
        switch (Shuffleboard.UNIVERSAL_MODE_CHOICE) {
            case "competition":
                autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier((stream) -> true? stream.filter(auto -> auto.getName().startsWith("comp_")): stream);
            case "testing":
                autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier((stream) -> true? stream.filter(auto -> auto.getName().startsWith("test_")): stream);
            case "allAutos":
                autoChooser = AutoBuilder.buildAutoChooser();
            default: //Default Puts All Commands. This is redundant because all the cases are hard coded.
                autoChooser = AutoBuilder.buildAutoChooser();
        }
        SmartDashboard.putData("Autonomous Choices", autoChooser);
    }

    public Command getAutonomousChoice() {
        return autoChooser.getSelected();
        //String choosenAuto = universalModeChooser.getSelected();
    }

    public void testingPIDTab(SwerveDrive swerveDrive){
        SwerveModule[] modules = swerveDrive.getModules();

        // Module Configuration
        SwerveModuleConfiguration frontLeftConfig = modules[0].configuration;
        SwerveModuleConfiguration frontRightConfig = modules[1].configuration;
        SwerveModuleConfiguration backLeftConfig = modules[2].configuration;
        SwerveModuleConfiguration backRightConfig = modules[3].configuration;

        SmartDashboard.putData("Front Left Drive", frontLeftConfig.velocityPIDF.createPIDController());
        SmartDashboard.putData("Front Left Angle", frontLeftConfig.anglePIDF.createPIDController());

        SmartDashboard.putData("Front Right Drive", frontRightConfig.velocityPIDF.createPIDController());
        SmartDashboard.putData("Front Right Angle", frontRightConfig.anglePIDF.createPIDController());

        SmartDashboard.putData("Back Left Drive", backLeftConfig.velocityPIDF.createPIDController());
        SmartDashboard.putData("Back Left Angle", backLeftConfig.anglePIDF.createPIDController());

        SmartDashboard.putData("Back Right Drive", backRightConfig.velocityPIDF.createPIDController());
        SmartDashboard.putData("Back Right Angle", backRightConfig.anglePIDF.createPIDController());
    }

}