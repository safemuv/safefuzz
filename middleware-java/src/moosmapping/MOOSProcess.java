package moosmapping;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class MOOSProcess extends MOOSElement {
	private List<MOOSBehaviour> behaviours = new ArrayList<MOOSBehaviour>();
	
	public MOOSProcess(String processName, MOOSCommunity parent) {
		this.parent = parent;
		this.processName = processName;
	}
	
	MOOSCommunity parent;
	String processName;
	
	private void generateMission(FileWriter f) {
		try {
			f.write("ServerPort = " + parent.getDBPort());
			f.flush();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	private void generateFiles(MOOSFiles mf) {	
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
	
	public void addBehaviour(MOOSBehaviour b) {
		behaviours.add(b);
	}
	
	public void generateCode(MOOSFiles fs) {
		generateFiles(fs);
		//generateBehaviourFile(fs);
	}
}
