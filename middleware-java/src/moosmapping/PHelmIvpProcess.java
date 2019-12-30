package moosmapping;

public class PHelmIvpProcess extends MOOSProcess {
	
	public PHelmIvpProcess(MOOSCommunity parent, String vehicleName, Integer minSpeed, Integer maxSpeed) {
		super("pHelmIvp", parent);
			
		// Links the Helm process to its behaviour file
		String parentBHVFile = parent.getBehaviourFileName();
		setProperty("AppTick", 4);
		setProperty("CommsTick", 4);
		setProperty("Behaviors", parentBHVFile);
		setProperty("Verbose", "false");
		setProperty("Domain", "course:0:359:360");
		setupBehaviours(vehicleName);
		
		// TODO: this encodes a speed limit. This should be factored out and set in the DSL
		// for the robot
		// Check these properties
		setProperty("Domain", "speed:0:" + minSpeed.toString() + ":" + maxSpeed.toString());
		//Domain       = speed:0:5:26
	}
	
	private void setupBehaviours(String vehicleName) {
		String defaultLoiterPos = "x=1,y=1";
		String defaultStartPos = "x=100,y=100";
		double loiterSpeed = 1.0;
		double loiterRadius = 5.0;
		double loiterNMRadius = 10.0;
		
		double waypointSpeed = 1.3;
		double waypointRadius = 3.0;
		double waypointNMRadius = 15.0;
		
		addBehaviour(new HelmBehaviourLoiter(this, vehicleName, defaultLoiterPos, loiterSpeed, loiterRadius, loiterNMRadius));
		addBehaviour(new HelmBehaviourStationKeep(this));
		addBehaviour(new HelmBehaviourWaypoint(this, vehicleName, defaultStartPos, waypointSpeed, waypointRadius, waypointNMRadius));
		
	}
}
