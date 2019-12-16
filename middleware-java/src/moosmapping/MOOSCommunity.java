package moosmapping;

import java.util.List;

public class MOOSCommunity {
	private String communityName;
	private int dbPort = 9000;
	private List<MOOSProcess> processes;
	
	public void generateCode(MOOSFiles mf) {
		// Generate a base mission file
		for (MOOSProcess p : processes) {
			p.generateCode(mf);
		}
	}
	
	public int getDBPort() {
		return dbPort;
	}
	
	public String getCommunityName() {
		return communityName;
	}
}
