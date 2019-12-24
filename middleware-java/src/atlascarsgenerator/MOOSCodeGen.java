package atlascarsgenerator;

import atlasdsl.Mission;
import carsmapping.CARSSimulation;
import moosmapping.*;

// FIX: maybe move to MOOSSimulation?

public class MOOSCodeGen extends CARSCodeGen {
	MOOSCodeGen(Mission m) {
		super(m);
	}
	
	public CARSSimulation convertDSL(Mission m) {
		MOOSSimulation moos = new MOOSSimulation();
		// This performs the translation from DSL objects to a MOOS mission definition
		// Steps: for each Robot, generate a MOOSCommunity
		// If the robot has a Sensor (sonar) then include the appropriate uFldHazardSensor
		// The shoreside also has to include the uFldSensor component
		// pShare component must be created if any messages are interchanged
		// Behaviours must include a Waypoint for each robot
		// ATLASDBWatch process must be created in the community for each robot
		// The watch variables depend on the mission goals (at least X,Y coordinates for now)
		
		// Later, this will be modified for any faults.
		return moos;
	}
}
