package frc.robot.autons;

import static frc.robot.Constants.AlgaeK.kLogTab;

import java.util.ArrayList;
import java.util.concurrent.Flow.Publisher;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autons.TrajsAndLocs.HPStation;
import frc.robot.autons.TrajsAndLocs.ReefLocs;
import frc.robot.autons.TrajsAndLocs.StartingLocs;
import frc.robot.subsystems.Elevator.EleHeight;
import frc.util.WaltLogger;

public class WaltAutonBuilder {
    public static NetworkTableInstance nte_inst = NetworkTableInstance.getDefault();
    public static NetworkTable nte_autonChooser = nte_inst.getTable("AutonChooser");

    // --- TOPICS
    // actions
    public static BooleanTopic BT_makeAuton = nte_inst.getBooleanTopic("/AutonChooser/makeAuton");
    public static BooleanTopic BT_clearAll = nte_inst.getBooleanTopic("/AutonChooser/clearAll");
    public static StringTopic ST_autonName = nte_inst.getStringTopic("/AutonChooser/autonName");

    // presets
    public static BooleanTopic BT_right = nte_inst.getBooleanTopic("/AutonChooser/right");
    public static BooleanTopic BT_left = nte_inst.getBooleanTopic("/AutonChooser/left");
    public static BooleanTopic BT_midOne = nte_inst.getBooleanTopic("/AutonChooser/midOne");    //midOne piece means G (left pole of far flat face)

    // info
    public static BooleanTopic BT_autonMade = nte_inst.getBooleanTopic("/AutonChooser/autonMade");
    
    // --- PUBLISHERS
    // actions
    public static BooleanPublisher pub_makeAuton;
    public static BooleanPublisher pub_clearAll;
    public static StringPublisher pub_autonName;

    // presets
    public static BooleanPublisher pub_right;
    public static BooleanPublisher pub_left;
    public static BooleanPublisher pub_midOne;

    //info
    public static BooleanPublisher pub_autonMade;

    // --- SUBSCRIBERS
    // actions
    public static BooleanSubscriber sub_makeAuton;
    public static BooleanSubscriber sub_clearAll;
    public static StringSubscriber sub_autonName;

    // presets
    public static BooleanSubscriber sub_right;
    public static BooleanSubscriber sub_left;
    public static BooleanSubscriber sub_midOne;
    
    // info
    public static BooleanSubscriber sub_autonMade;

    public static void initialize() {
        // PUBLISHING TO TOPICS
        pub_makeAuton = BT_makeAuton.publish();
        pub_clearAll = BT_clearAll.publish();
        pub_autonName = ST_autonName.publish();
        
        pub_right = BT_right.publish();
        pub_left = BT_left.publish();
        pub_midOne = BT_midOne.publish();

        pub_autonMade = BT_autonMade.publish();

        // SUBSCRIBING TO TOPICS
        sub_makeAuton = BT_makeAuton.subscribe(false);
        sub_clearAll = BT_clearAll.subscribe(false);
        sub_autonName = ST_autonName.subscribe("No Auton Made");

        sub_right = BT_right.subscribe(false);
        sub_left = BT_left.subscribe(false);
        sub_midOne = BT_midOne.subscribe(false);

        sub_autonMade = BT_autonMade.subscribe(false);

        // PUBLISHER DEFAULTS
        pub_makeAuton.setDefault(false);
        pub_clearAll.setDefault(false);
        pub_autonName.setDefault("No Auton Made");

        pub_right.setDefault(false);
        pub_left.setDefault(false);
        pub_midOne.setDefault(false);

        pub_autonMade.setDefault(false);
    }
}