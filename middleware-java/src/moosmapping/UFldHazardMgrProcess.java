package moosmapping;

public class UFldHazardMgrProcess extends MOOSProcess {
	public UFldHazardMgrProcess(MOOSCommunity parent, String robotName, int swathWidth, double detectionProb) {
		super("uFldHazardMgr", parent);
		setProperty("swath_width", 25);
		setProperty("sensor_pd", detectionProb);
		setProperty("report_name", robotName);
	}
}
