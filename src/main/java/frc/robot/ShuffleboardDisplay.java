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
    private final SendableChooser<String> universalModeChooser = new SendableChooser<>();
    private final static SendableChooser<String> controlChooser = new SendableChooser<>();
        private SendableChooser<Command> autoChooser;
        //private GenericEntry genericEntryTest = mainTab.add("Generic",0).getEntry(); // Example: Putting Data on other tabs
    
    
        public void initiateDisplay(){
            universalModeChooser.setDefaultOption("Competition Autos","competiton");
            universalModeChooser.addOption("Testing: Test Autos", "testing");
            universalModeChooser.addOption("Testing: All Autos", "allAutos");
            SmartDashboard.putData("Universal Mode Chooser", universalModeChooser);
            //genericEntryTest.setDouble(12.3);
        }
    
    
        public void initializeAutoChooser(){
            if(ShuffleboardConstants.UNIVERSAL_MODE_CHOICE.equalsIgnoreCase("competition")) {
                    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier((stream) -> true? stream.filter(auto -> auto.getName().startsWith("comp_")): stream);
            } else if(ShuffleboardConstants.UNIVERSAL_MODE_CHOICE.equalsIgnoreCase("testing")){
                    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier((stream) -> true? stream.filter(auto -> auto.getName().startsWith("test_")): stream);
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
}