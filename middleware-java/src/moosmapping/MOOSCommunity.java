package moosmapping;

import java.util.ArrayList;
import java.util.List;

public class MOOSCommunity {
	private static int dbCounter = 9000;
	private String communityName;
	private int dbPort = dbCounter++;
	private List<MOOSProcess> processes = new ArrayList<MOOSProcess>();
	
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
	
	public MOOSCommunity(String communityName) {
		this.communityName = communityName;
	}
	public MOOSCommunity(String communityName, int dbPort) {
		this.communityName = communityName;
		this.dbPort = dbPort;
	}
	
	public void addProcess(MOOSProcess p) {
		processes.add(p);
	}
	
}
