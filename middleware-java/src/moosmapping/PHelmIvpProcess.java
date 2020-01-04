package moosmapping;

import atlasdsl.*;

public class PHelmIvpProcess extends MOOSProcess {
	
	public PHelmIvpProcess(MOOSCommunity parent, Point startPos, String vehicleName, Integer minSpeed, Integer maxSpeed) {
		super("pHelmIvp", parent);
			
		// Links the Helm process to its behaviour file
		String parentBHVFile = parent.getBehaviourFileName();
		setProperty("AppTick", 4);
		setProperty("CommsTick", 4);
		setProperty("Behaviors", parentBHVFile);
		setProperty("Verbose", "false");
		setProperty("Domain", "course:0:359:360");
		
		String pointAsString = startPos.toString();
		
		setupBehaviours(vehicleName, pointAsString);
		setupBehaviourVars();
		
		// TODO: this encodes a speed limit. This should be factored out and set in the DSL
		// for the robot
		// Check these properties
		setProperty("Domain", "speed:0:" + minSpeed.toString() + ":" + maxSpeed.toString());
		//Domain       = speed:0:5:26
		
		
	}
	
	private void setupBehaviourVars() {

		moosBehaviourInitVals.put("DEPLOY", false);
		moosBehaviourInitVals.put("RETURN", false);
		moosBehaviourInitVals.put("STATION_KEEP", false);
		moosBehaviourInitVals.put("LOITER", true);
		// TODO: this should be conditionally set if an avoidance goal is present
		moosBehaviourInitVals.put("AVOID", true);
				
		MOOSSetModeDetails msd1 = new MOOSSetModeDetails("ACTIVE", "INACTIVE");
		msd1.setProperty("DEPLOY", true);
		setModeProperties.put("ACTIVE", msd1);
		
		MOOSSetModeDetails msd2 = new MOOSSetModeDetails("STATION-KEEPING");
		msd2.setProperty("MODE", "ACTIVE");
		msd2.setProperty("STATION_KEEP", true);
		setModeProperties.put("STATION-KEEPING", msd2);
		
		MOOSSetModeDetails msd3 = new MOOSSetModeDetails("RETURNING");
		msd3.setProperty("MODE", "ACTIVE");
		msd3.setProperty("RETURN", true);
		setModeProperties.put("RETURNING", msd3);
		
		MOOSSetModeDetails msd4 = new MOOSSetModeDetails("LOITERING");
		msd4.setProperty("MODE", "ACTIVE");
		msd4.setProperty("LOITER", true);
		setModeProperties.put("LOITERING", msd4);
	}

	
	private void setupBehaviours(String vehicleName, String startPos) {
		double loiterSpeed = 1.0;
		double loiterRadius = 5.0;
		double loiterNMRadius = 10.0;
		
		double waypointSpeed = 1.3;
		double waypointRadius = 3.0;
		double waypointNMRadius = 15.0;
		
		addBehaviour(new HelmBehaviourLoiter(this, vehicleName, startPos, loiterSpeed, loiterRadius, loiterNMRadius));
		addBehaviour(new HelmBehaviourStationKeep(this));
		addBehaviour(new HelmBehaviourWaypoint(this, vehicleName, startPos, waypointSpeed, waypointRadius, waypointNMRadius));
	}
}
