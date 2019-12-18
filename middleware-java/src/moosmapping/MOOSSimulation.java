package moosmapping;

import java.util.ArrayList;
import java.util.List;

import carsmapping.CARSMapping;

public class MOOSSimulation implements CARSMapping {
	private List<MOOSCommunity> communities = new ArrayList<MOOSCommunity>();
	MOOSLaunchScript ls;
	
	public void generateCARSInterface(String baseDirectory) {
		MOOSFiles mf = new MOOSFiles(baseDirectory);
		// Set up the launch script
		ls.generateScript(mf);
		
		for (MOOSCommunity c : communities) { 
			// This generates the code for every MOOS vehicle 
			// in the simulation
			c.generateCode(mf);
		}
		
		mf.closeAllFiles();
	}
	
	public MOOSSimulation() {
		ls = new MOOSLaunchScript();
	}
	
	public void addCommunity(MOOSCommunity c) {
		communities.add(c);
	}
}
