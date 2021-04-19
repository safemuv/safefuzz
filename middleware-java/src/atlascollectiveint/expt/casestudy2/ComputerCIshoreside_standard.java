package atlascollectiveint.expt.casestudy2;

import atlascollectiveint.api.*;
import atlascollectiveint.logging.CollectiveIntLog;
import atlassharedclasses.*;

import java.lang.Double;
import java.lang.String;

public class ComputerCIshoreside_standard {
	
	static final double TIME_BEFORE_SWITCHING = 10000;
	static final double TIME_TO_AUTO_RETURN = 1050;
	
	static Region region1 = new Region(new Point(170, -100), new Point(209, -60));
	static Region region2 = new Region(new Point(-75, -100), new Point(-35, -60));
	static Region gildaRegion = region1;
	static Region henryRegion = region2;
	
	public static void alternateRegions() {
		Region tmp = gildaRegion;
		gildaRegion = henryRegion;
		henryRegion = tmp;
	}
	
	public static void setVehicleRegions() {
		API.setPatrolAroundRegion("gilda", gildaRegion, 10,
				"UUV_COORDINATE_UPDATE_INITIAL_GILDA");
		API.setPatrolAroundRegion("henry", henryRegion, 10,
				"UUV_COORDINATE_UPDATE_INITIAL_HENRY");
	}
	
	public static void init() {
		API.startVehicle("gilda");
		API.startVehicle("henry");
		setVehicleRegions();
		
		PeriodicTimer tSwitchRegion = new PeriodicTimer(TIME_BEFORE_SWITCHING, (t -> {
			alternateRegions();
			setVehicleRegions();
		}));
		
		// Set all the robots to return at a given time
		OneOffTimer tReturn = OneOffTimer.atTime(TIME_TO_AUTO_RETURN, (t -> {
			API.returnHome("gilda");
			API.returnHome("henry");
		}));
		
		API.registerTimer("returnHome", tReturn);
		API.registerTimer("switchRegions", tSwitchRegion);
	}


	public static void SONARDetectionHook(SensorDetection detection, String robotName) {
		CollectiveIntLog.logCI("SONARDetectionHook");
	}

	public static void GPS_POSITIONDetectionHook(Double x, Double y, String robotName) {

	}

	public static void CAMERADetectionHook(SensorDetection detection, String robotName) {
		CollectiveIntLog.logCI("CAMERADetectionHook");
	}
	
	public static void EnergyUpdateHook(EnergyUpdate energyUpdate, String robotName) {
		//System.out.println("EnergyUpdateHook - energy value is " + energyUpdate.getEnergyValue());
	}
	
	public static void BehaviourVariableHook(String key, String value, String robotName) {
		System.out.println("BehaviourVariableHook: robotName = " + robotName + ",key = " + key + ",value=" + value);
	}
}