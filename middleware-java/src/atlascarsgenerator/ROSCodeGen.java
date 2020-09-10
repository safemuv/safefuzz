package atlascarsgenerator;

import carsmapping.CARSSimulation;
import carsspecific.moos.moosmapping.*;
import carsspecific.ros.rosmapping.ROSSimulation;

import java.util.ArrayList;
import java.util.List;

import atlasdsl.*;
import atlassharedclasses.Point;

public class ROSCodeGen extends CARSCodeGen {
	// TODO: how to specify the sensor behaviour
	public ROSCodeGen(Mission m) {
		super(m);
	}
	
	// This performs the necessary processing to add sensors to the robots
	// If the robot has a Sensor (sonar) then include the appropriate uFldHazardSensor
	// The shoreside also has to include the uFldHazardMgr component
	private void setupSensors(Mission mission, MOOSSimulation moossim, List<String> moosSharedVars) {
		// The robots which have sensors have to have an appropriate uFldHazardMgr? process for them
		// Shoreside or other monitoring computers have to have uFldHazardSensor
		// variables have to be added
		// if an unexpected sensor is present, raise an exception?
		
	}
	
	// This performs the necessary processing to set up actuators
	private void setupActuators(Mission mission, MOOSSimulation moossim, List<String> moosSharedVars) {
		
	}
	
	private List<String> varsForMissionGoals(Mission mission) {
		// TODO: process the mission, translate the mission goals to variables
		List<String> vs = new ArrayList<String>();
		// Assume all missions monitor the X,Y coordinates of robots
		// TODO: check and add other necessary variables here
		vs.add("VAR_X");
		vs.add("VAR_Y");
		vs.add("DB_UPTIME");
		return vs;
	}
	
	private List<String> varsForCI(Mission mission) {
		return new ArrayList<>();
	}
	
	// Creates an ATLASDBWatch component to watch the given variables
	private void createATLASLink(MOOSCommunity c, List<String> middleWareVars, int port) {
		System.out.println("createATLASLink");
		//ATLASWatchProcess dbwatch = new ATLASWatchProcess(c, c.getCommunityName());
		
		ATLASInterfaceProcess dbwatch = new ATLASInterfaceProcess(c, c.getCommunityName());
		for (String v : middleWareVars) {
			dbwatch.addWatchVariable(v);
		}
		c.addProcess(dbwatch);
	}
	
	public CARSSimulation convertDSL(Mission mission) throws ConversionFailed {
		CARSSimulation rossim = new ROSSimulation();
		return rossim;	
	}}
