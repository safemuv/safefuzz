package moosmapping;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class MOOSCommunity {
	private static int dbCounter = 9000;
	private String communityName;
	private int dbPort = dbCounter++;
	private List<MOOSProcess> processes = new ArrayList<MOOSProcess>();
	
	private void genBasicMissionParams(FileWriter missionFile) throws IOException {
		missionFile.write("ServerHost = localhost\n");
		missionFile.write("ServerPort = " + dbPort + "\n");
		missionFile.write("Simulator = true\n");
		missionFile.write("community = " + communityName + "\n");
		// maybe other things e.g. "MOOSTimeWarp" etc here
	}
	
	private void genANTLERBlock(FileWriter missionFile) throws IOException {
		missionFile.write("\nProcessConfig = ANTLER {\n");
		String consoleStatus = " @ NewConsole = false";
		for (MOOSProcess p : processes) {
			missionFile.write("Run = " + p.processName + consoleStatus + "\n");
		}
		missionFile.write("}\n\n");
	}
	
	private void generateMissionFile(FileWriter missionFile) throws IOException {
		genBasicMissionParams(missionFile);
		genANTLERBlock(missionFile);
		for (MOOSProcess p : processes) {
			p.generateConfigBlock(missionFile);
		}
	}
	
	private void generateBehaviourFile(FileWriter behaviourFile) throws IOException {
		for (MOOSProcess p : processes) {
			p.generateBehavioursForProcess(behaviourFile);
		}
	}
	
	public void generateCode(MOOSFiles mf) {
		// Generate a base mission file
				
		String missionFileName = "targ_" + communityName + ".moos";
		String behaviourFileName = "targ_" + communityName + ".bhv";
		try {
			FileWriter missionFile = mf.getOpenFile(missionFileName);
			generateMissionFile(missionFile);
			FileWriter behaviourFile = mf.getOpenFile(behaviourFileName);
			generateBehaviourFile(behaviourFile);
			
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
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
