package moosmapping;

import moosmapping.*;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class MOOSProcess extends MOOSElement {
	private List<MOOSBehaviour> behaviours = new ArrayList<MOOSBehaviour>();
	private static int defaultCommsTick = 2;
	private static int defaultAppTick = 2;
	
	public MOOSProcess(String processName, MOOSCommunity parent) {
		this.parent = parent;
		this.processName = processName;
		// Set standard properties, e.g. APPTick
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
	
	public void generateBehavioursForProcess(FileWriter behaviourFile) throws IOException {	
		for (MOOSBehaviour b : behaviours)
			b.generateCode(behaviourFile);
	}
	
	public void addBehaviour(MOOSBehaviour b) {
		behaviours.add(b);
	}
}
