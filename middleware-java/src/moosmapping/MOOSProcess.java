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
	
	public void generateConfigBlock(FileWriter missionFile) throws IOException {
		missionFile.write("ProcessConfig = " + processName + "\n{\n");
		// TODO: generate all the properties here!
		missionFile.write("...TODO: properties here...");
		missionFile.write("}\n");
	}
	
	public void generateBehavioursForProcess(FileWriter behaviourFile) throws IOException {	
		for (MOOSBehaviour b : behaviours)
			b.generateCode(behaviourFile);
	}
	
	public void addBehaviour(MOOSBehaviour b) {
		behaviours.add(b);
	}
}
