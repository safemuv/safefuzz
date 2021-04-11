package carsspecific.moos.moosmapping;

import atlassharedclasses.Point;

public class PHelmIvpProcess extends MOOSProcess {
	
	private double startSpeed = 1.0;
	// The number of divisions into which valid speeds is divided - e.g. 26 permits granularity of 0.05 over the range 0 -> 5 m/s 
	private int NUM_SPEED_DIVISIONS = 26;
	
	private final double EXTRA_DEPTH_SHIFT = 0.01;
	
	private boolean verboseHelm = true;
	
	public PHelmIvpProcess(MOOSCommunity parent, Point startPos, String vehicleName, double startSpeed, double maxSpeed, double maxDepth) {
		super("pHelmIvP", parent);
		this.startSpeed = startSpeed;
			
		// Links the Helm process to its behaviour file
		String parentBHVFile = parent.getBehaviourFileName();
		resetProperty("AppTick", 4);
		resetProperty("CommsTick", 4);
		setProperty("Verbose", verboseHelm);
		setProperty("Behaviors", parentBHVFile);
		setProperty("Domain", "course:0:359:360");
		
		int depthSteps = Math.min(2, (int)(Math.ceil(maxDepth) + 2));
		setProperty("Domain", "depth:0:" + Double.toString(EXTRA_DEPTH_SHIFT + maxDepth) + ":" + Integer.toString(depthSteps) + ":optional");
		
		setupBehaviours(vehicleName, startPos, maxDepth);
		setupBehaviourVars();
		
		// The domain has to be set using the maximum speed here
		Double minSpeed = 0.0;
		Double maxSpeed_boxed = maxSpeed;
		Integer numSpeedPoints = NUM_SPEED_DIVISIONS;
		setProperty("Domain", "speed:" + minSpeed.toString() + ":" + maxSpeed_boxed.toString() + ":" + numSpeedPoints.toString());
	}
	
	private void setupBehaviourVars() {
		moosBehaviourInitVals.put("DEPLOY", true);
		moosBehaviourInitVals.put("WAYPOINT", false);
		moosBehaviourInitVals.put("STATION_KEEP", false);
		//moosBehaviourInitVals.put("LOITER", true);
		moosBehaviourInitVals.put("CONSTHEADING", false);
		moosBehaviourInitVals.put("CONSTDEPTH", true);
		// TODO: this should be conditionally set if an avoidance goal is present
		moosBehaviourInitVals.put("AVOID", true);
	}
	
	private void setupBehaviours(String vehicleName, Point startPos, double maxDepth) {
		String startPosAsString = startPos.toString();
		
		double loiterSpeed = startSpeed;
		double loiterRadius = 5.0;
		double loiterNMRadius = 10.0;
		
		double waypointSpeed = startSpeed;
		double waypointRadius = 3.0;
		double waypointNMRadius = 15.0;
		
		//addBehaviour(new HelmBehaviourLoiter(this, vehicleName, startPosAsString, loiterSpeed, loiterRadius, loiterNMRadius));
		addBehaviour(new HelmBehaviourAvoidance(this, vehicleName));
		// StationKeeping is not needed for current scenario
		addBehaviour(new HelmBehaviourStationKeep(this, startPos));
		addBehaviour(new HelmBehaviourWaypoint(this, vehicleName, startPos, waypointSpeed, waypointRadius, waypointNMRadius));
		addBehaviour(new HelmBehaviourConstantHeading(this));
		addBehaviour(new HelmBehaviourConstantSpeed(this));
		addBehaviour(new HelmBehaviourConstantDepth(this, maxDepth));
	}
}
