package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShuffleboardDisplay {

    private static ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Main");
    private static GenericEntry genericEntryTest = shuffleboardTab.add("Generic",0).getEntry();

    public static void update(){
        genericEntryTest.setDouble(12.3);
        //SmartDashboard.putNumber("Test",1);
    }
}

