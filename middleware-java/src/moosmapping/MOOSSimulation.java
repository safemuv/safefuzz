package moosmapping;

import carsmapping.CARSMapping;

public class MOOSSimulation implements CARSMapping {
	MOOSCommunity s;
	MOOSLaunchScript ls;

	public void generateCARSInterface(String baseDirectory) {
		MOOSFiles mf = new MOOSFiles(baseDirectory);
		// Set up the launch script
		ls.generateScript(mf);
	}
}
