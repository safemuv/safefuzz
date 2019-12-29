package moosmapping;

public class RobotCommunity extends MOOSCommunity {
	// TODO: Add custom behaviours in constructor - must include a Waypoint for each
	// robot to be updated by the collective intelligence
	
	private int defaultMinSpeed = 5;
	private int defaultMaxSpeed = 26;
	
	private void createRobotProcesses() {
		// All Robots include a Logger...
		addProcess(new PLoggerProcess(this, "LOG_" + communityName, false));
		// and uSimMarine and Helm
		// TODO: check parameters for the processes
		addProcess(new USimMarineProcess(this));
		addProcess(new PMarinePIDProcess(this));
		
		addProcess(new PHelmIvpProcess(this, defaultMinSpeed, defaultMaxSpeed));
		addProcess(new PHostinfoProcess(this));
		addProcess(new PBasicContactMgrProcess(this));
		addProcess(new PNodeReporterProcess(this));
	}
	
	public RobotCommunity(String robotName) {
		super(robotName);
		createRobotProcesses();
	}
}
