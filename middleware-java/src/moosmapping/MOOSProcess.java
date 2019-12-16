package moosmapping;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;

public class MOOSProcess extends MOOSElement {
	private List<MOOSBehaviour> behaviours;
	
	MOOSCommunity parent;
	String processName;
	
	private void generateMission(FileWriter f) {
		try {
			f.write("ServerPort = " + parent.getDBPort());
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	private void generateBehaviourFile(MOOSFiles mf) {	
		FileWriter bhvFile, missionFile;
		try {
			bhvFile = mf.createOpenFile(processName + ".bhv");
			missionFile = mf.createOpenFile(processName + ".moos");
			generateMission(missionFile);
			for (MOOSBehaviour b : behaviours)
				// Need to put the basic code here 
				b.generateCode(bhvFile);
			
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	private void generateMissionFile(MOOSFiles fs) {
		// The process will generate a plugin in the core file?
	}
	
	public void generateCode(MOOSFiles fs) {
		generateMissionFile(fs);
		generateBehaviourFile(fs);
	}
}
