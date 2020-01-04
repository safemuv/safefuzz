package atlascarsgenerator;

import carsmapping.CARSSimulation;
import moosmapping.*;

import java.util.ArrayList;
import java.util.List;

import atlasdsl.*;

// FIX: maybe move to MOOSSimulation?

public class MOOSCodeGen extends CARSCodeGen {
	// TODO: how to specify the sensor behaviour
	public MOOSCodeGen(Mission m) {
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
		return vs;
	}
	
	private List<String> varsForCI(Mission mission) {
		return new ArrayList<>();
	}
	
	// Creates an ATLASDBWatch component to watch the given variables
	private void createATLASLink(MOOSCommunity c, List<String> middleWareVars, int port) {
		
	}
	
	public CARSSimulation convertDSL(Mission mission) throws ConversionFailed {
		// Names of variables to be shared via pShare
		List<String> moosSharedVars = new ArrayList<String>();
		List<String> middlewareVars = varsForMissionGoals(mission);
		List<String> collectiveIntelVars = varsForCI(mission); 
		
		int atlasPort = 61613;
		try {
		
			MOOSSimulation moossim = new MOOSSimulation();
			// This performs the translation from DSL objects to a MOOS mission definition
		// Firstly: for each Robot, generate a MOOSCommunity
		for (Robot r : mission.getAllRobots()) {
			Point startPos = r.getPointComponentProperty("startLocation");
			MOOSCommunity rprocess = new RobotCommunity(moossim, r, startPos);
			moossim.addCommunity(rprocess);
			System.out.println("Adding community for robot: " + r.getName());
			//TODO: AvoidCollision should be added to the new Robot's Helm behaviours when an an avoidance goal exists
		}
		
		// This currently encodes the assumption of one shoreside computer
		// performing system monitoring/management of CI
		// TODO: Should be made more flexible - we could have multiple
		// computers involved
		if (mission.includesComputer()) {
			MOOSCommunity shoreside = new ComputerCommunity(moossim,"shoreside");
			moossim.addCommunity(shoreside);
			System.out.println("Adding community for fixed computer");
		}
				
		// Setup the sensors and actuators
		setupSensors(mission, moossim, moosSharedVars);
		setupActuators(mission, moossim, moosSharedVars);	

		for (MOOSCommunity c : moossim.getAllCommunities()) {
			// pShare component must be notified if any messages are interchanged
			// For now, just notify all communities of all messages
			// TODO: think about restricting pShare message transfer
			c.registerSharedVars(moosSharedVars);
			
			// Do we also have to register NodeBroker here?
			
			// ATLASDBWatch process must be created in the each community 
			// to watch the given variables for the middleware
			createATLASLink(c, middlewareVars, atlasPort);
		}
		
		// TODO: this returns a MOOS community without any faults
		// There will be a later stage to modify the community to inject faults.
		return moossim;
		
		} catch (MissingProperty mp) {
			System.out.println("Conversion failed: component " + mp.getComponent() + " is missing property " + mp.getPropertyName() + "...");
			mp.printStackTrace();
			throw new ConversionFailed();
		}
		
	}
}
