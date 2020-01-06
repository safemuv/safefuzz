package moosmapping;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import carsmapping.*;

public class MOOSSimulation implements CARSSimulation {
	private int dbPortCounter = 9000;
	
	private List<MOOSCommunity> communities = new ArrayList<MOOSCommunity>();
	
	private void generateLaunchScript(MOOSFiles mf) throws IOException {
		// Append a string with ampersand to launch all processes simultaneously
		String launchSeperatelyStr = " &";
		FileWriter missionFile = mf.getOpenFile("launch.sh");
		missionFile.write("#!/bin/sh\n\n");
		for (MOOSCommunity c : communities) {
			missionFile.write("pAntler " + c.getMissionFileName() + launchSeperatelyStr + "\n");
		}
	}
	
	public int nextDBPortCounter() {
		return dbPortCounter++;
	}
	
	public void generateCARSInterface(String baseDirectory) {
		MOOSFiles mf = new MOOSFiles(baseDirectory);
		
		try {
			generateLaunchScript(mf);
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		for (MOOSCommunity c : communities) { 
			// This generates the code for every MOOS vehicle 
			// in the simulation
			c.generateCode(mf);
		}
		
		mf.closeAllFiles();
	}
	
	public MOOSSimulation() {
		
	}
	
	public void addCommunity(MOOSCommunity c) {
		communities.add(c);
	}

	public List<MOOSCommunity> getAllCommunities() {
		return communities;
	}
}
