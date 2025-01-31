package frc.robot;
import frc.robot.Constants;
import frc.robot.Constants.ShuffleboardConstants;

import javax.lang.model.type.NullType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; //Some things have to go through SmartDashboard
import edu.wpi.first.wpilibj2.command.Command;

public class ShuffleboardDisplay {

    private static ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Main");
    private String m_autoSelected;
    private final SendableChooser<String> autonomousChooser = new SendableChooser<>();
    private static GenericEntry genericEntryTest = shuffleboardTab.add("Generic",0).getEntry();


    public void initiateDisplay(){
        SmartDashboard.putString("Auto Mode", "None");
        autonomousChooser.setDefaultOption(ShuffleboardConstants.AUTONOMOUS_CHOOSER_DEFAULT_TEXT, ShuffleboardConstants.AUTONOMOUS_CHOOSER_DEFAULT_OBJECT);
        for(int i=0; i<ShuffleboardConstants.AUTONOMOUS_CHOOSER_OPTIONS_TEXT.length; i++){
            autonomousChooser.addOption(ShuffleboardConstants.AUTONOMOUS_CHOOSER_OPTIONS_TEXT[i], ShuffleboardConstants.AUTONOMOUS_CHOOSER_OPTIONS_OBJECTS[i]); 
        }
        SmartDashboard.putData("Autonomous choices", autonomousChooser);
        genericEntryTest.setDouble(12.3);
        //SmartDashboard.putNumber("Test",1);
    }

    public String getAutonomousCommand() {
        return autonomousChooser.getSelected();
    }
}

