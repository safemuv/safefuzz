package moosmapping;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class MOOSProcess extends MOOSElement {
	private List<MOOSBehaviour> behaviours = new ArrayList<MOOSBehaviour>();
	// Used to generate the initialize lines at start of behaviour file
	private Map<String,Object> moosBehaviourInitVals = new HashMap<String,Object>();
	
	
	private static int defaultCommsTick = 2;
	private static int defaultAppTick = 2;
	
	public MOOSProcess(String processName, MOOSCommunity parent) {
		this.parent = parent;
		this.processName = processName;
		this.setProperty("AppTick", defaultAppTick);
		this.setProperty("CommsTick", defaultCommsTick);
	}
	
	MOOSCommunity parent;
	String processName;
	
	public void generateConfigBlock(FileWriter missionFile) throws IOException {
		missionFile.write("ProcessConfig = " + processName + "\n{\n");
		writePropertiesDefault(missionFile);
		missionFile.write("}\n\n");
	}
	
	private void generateBehaviourInitSection(FileWriter bhvFile) throws IOException {
		// Generate the initialization values
		for (Map.Entry<String,Object> entry : moosBehaviourInitVals.entrySet()) {
			String name = entry.getKey();
			Object value = entry.getValue();
			bhvFile.write("initialize " + name + " = " + value);
		}
		// TODO: mapping between modes and variables
	}
	
	public void generateBehavioursForProcess(FileWriter behaviourFile) throws IOException {	
		generateBehaviourInitSection(behaviourFile);
		for (MOOSBehaviour b : behaviours)
			b.generateCode(behaviourFile);
	}
	
	public void addBehaviour(MOOSBehaviour b) {
		behaviours.add(b);
	}
}
