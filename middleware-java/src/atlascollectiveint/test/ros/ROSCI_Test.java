package atlascollectiveint.test.ros;

import atlasdsl.MissingProperty;
import atlasdsl.Mission;
import atlasdsl.Robot;
import atlassharedclasses.*;
import java.lang.Double;
import java.lang.String;
import java.util.ArrayList;
import java.util.List;

public class ROSCI_Test {

	private static Mission mission;

	public static void init() {
		System.out.println("init");
		
		
	}
	
	public static void SONARDetectionHook(SensorDetection d, String robotName) {
		// Update the robot position notification
		//robotLocations.put(robotName, new Point(x, y));
	}

	public static void GPS_POSITIONDetectionHook(Double x, Double y, String robotName) {
		// Update the robot position notification
		//robotLocations.put(robotName, new Point(x, y));
		System.out.println("GPS POS = " + x + "," + y);
	}

	public static void CAMERADetectionHook(SensorDetection detection, String robotName) {

	}

	public static void EnergyUpdateHook(EnergyUpdate energyUpdate, String robotName) {

	}

	public static void BehaviourVariableHook(String key, String value, String robotName_uc, Double timeNow) {

	}
}
