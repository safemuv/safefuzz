package moosmapping;

public class PHelmIvpProcess extends MOOSProcess {
	public PHelmIvpProcess(MOOSCommunity parent, Integer minSpeed, Integer maxSpeed) {
		super("pHelmIvp", parent);
		
		String parentBHVFile = parent.getBehaviourFileName();
		setProperty("AppTick", 4);
		setProperty("CommsTick", 4);
		setProperty("Behaviors", parentBHVFile);
		setProperty("Verbose", "false");
		setProperty("Domain", "course:0:359:360");
		
		// TODO: this encodes a speed limit. This should be factored out and set in the DSL
		// for the robot
		// Check these properties
		setProperty("Domain", "speed:0:" + minSpeed.toString() + ":" + maxSpeed.toString());
		//Domain       = speed:0:5:26
	}
}
