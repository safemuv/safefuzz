package moosmapping;

public class ComputerCommunity extends MOOSCommunity {
	public ComputerCommunity(MOOSSimulation sim, String robotName) {
		super(sim,robotName);
		addProcess(new PMarineViewerProcess(this));
	}
}
