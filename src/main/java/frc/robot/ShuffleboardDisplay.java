package frc.robot;
import frc.robot.Constants;
import frc.robot.Constants.ShuffleboardConstants;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.motors.SwerveMotor;
import swervelib.parser.SwerveModuleConfiguration;

import java.util.Map;

import javax.lang.model.type.NullType;
import javax.xml.crypto.dsig.spec.C14NMethodParameterSpec;

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

    //private ShuffleboardTab mainTab = Shuffleboard.getTab("Main"); // Example: Making more tabs pn the dashboard
    //private ShuffleboardTab PID_Tab = Shuffleboard.getTab("PID_Tab");
    private final SendableChooser<String> universalModeChooser = new SendableChooser<>(); // Dropdown Selection; NOTE: The modeChooser doesn't work because we need autos loaded before we can make a selection.
    private final static SendableChooser<String> controlChooser = new SendableChooser<>(); // Auto Path Dropdown
    private SendableChooser<Command> autoChooser;
    //private GenericEntry genericEntryTest = mainTab.add("Generic",0).getEntry(); // Example: Putting Data on other tabs
    
    public void initiateDisplay(){
        universalModeChooser.setDefaultOption("Competition Autos","competiton");
        universalModeChooser.addOption("Testing: Test Autos", "testing");
        universalModeChooser.addOption("Testing: All Autos", "allAutos");
        SmartDashboard.putData("Universal Mode Chooser", universalModeChooser);
        SmartDashboard.putString("Auton_Input_Code", "None");
        //genericEntryTest.setDouble(12.3);
    }


    public void initializeAutoChooser(){ // Puts auto commands into the dropdown. It returns the actual command based on the choice, makes things easy.
        if(ShuffleboardConstants.UNIVERSAL_MODE_CHOICE.equalsIgnoreCase("competition")) {
                autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier((stream) -> true? stream.filter(auto -> auto.getName().startsWith("comp_")): stream); // Display autos who's names contains the specified prefix.
        } else if(ShuffleboardConstants.UNIVERSAL_MODE_CHOICE.equalsIgnoreCase("testing")){
                autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier((stream) -> true? stream.filter(auto -> auto.getName().startsWith("test_")): stream); // ^
        } else if(ShuffleboardConstants.UNIVERSAL_MODE_CHOICE.equalsIgnoreCase("allAutos")){
                autoChooser = AutoBuilder.buildAutoChooser();
        } else { //Default Puts All Commands. This is redundant because all the cases are hard coded.
                autoChooser = AutoBuilder.buildAutoChooser(); 
        }
        SmartDashboard.putData("Autonomous Choices", autoChooser);
    }

    public Command getAutonomousChoice() {
        return autoChooser.getSelected();
    }

    public static String getControlModeChoice() {
        return controlChooser.getSelected();
    }

    public String getAutonInputCode() {
        return SmartDashboard.getString("Auton_Input_Code", "None"); // Gets the Auton Input Code for pre-set movements
    }
}