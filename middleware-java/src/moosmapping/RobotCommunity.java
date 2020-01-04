package moosmapping;

import atlasdsl.*;

public class RobotCommunity extends MOOSCommunity {
	// TODO: Add custom behaviours in constructor - must include a Waypoint for each
	// robot to be updated by the collective intelligence
	
	private int defaultMinSpeed = 5;
	private int defaultMaxSpeed = 26;
	
	public RobotCommunity(MOOSSimulation sim, Robot robot, Point startPos) {
		super(sim,robot.getName());
		
		// All Robots include a Logger...
		addProcess(new PLoggerProcess(this, "LOG_" + communityName, false));
		// and uSimMarine and Helm
		addProcess(new USimMarineProcess(this));
		addProcess(new PHelmIvpProcess(this, startPos, robot.getName(), defaultMinSpeed, defaultMaxSpeed));
		addProcess(new PMarinePIDProcess(this));
		addProcess(new PHostinfoProcess(this));
		addProcess(new PBasicContactMgrProcess(this));
		addProcess(new PNodeReporterProcess(this, robot));
	}
}
