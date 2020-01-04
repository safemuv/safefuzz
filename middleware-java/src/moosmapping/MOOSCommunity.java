package moosmapping;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class MOOSCommunity {
	protected String communityName;
	private int dbPort;
	private List<MOOSProcess> processes = new ArrayList<MOOSProcess>();
	private List<String> sharedVars = new ArrayList<String>();
	
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
	
	public String getMissionFileName() {
		return "targ_" + communityName + ".moos";
	}
	
	public String getBehaviourFileName() {
		return "targ_" + communityName + ".bhv";
	}
	
	public void generateCode(MOOSFiles mf) {
		// Generate mission and behaviour files
		try {
			FileWriter missionFile = mf.getOpenFile(getMissionFileName());
			generateMissionFile(missionFile);
			FileWriter behaviourFile = mf.getOpenFile(getBehaviourFileName());
			generateBehaviourFile(behaviourFile);
			
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	public int getDBPort() {
		return dbPort;
	}
	
	public String getCommunityName() {
		return communityName;
	}
	
	private void addProcessWatch() {
		addProcess(new UProcessWatch(this));
	}
	
	public MOOSCommunity(MOOSSimulation sim, String communityName) {
		this.communityName = communityName;
		this.dbPort = sim.nextDBPortCounter();
		addProcessWatch();
	}
	
	public MOOSCommunity(MOOSSimulation sim, String communityName, int dbPort) {
		this.communityName = communityName;
		this.dbPort = dbPort;
		addProcessWatch();
	}
	
	public MOOSCommunity(MOOSSimulation sim, String communityName, int dbPort, boolean watchProcess) {
		this.communityName = communityName;
		this.dbPort = dbPort;
		addProcessWatch();
		if (watchProcess) {
			addProcessWatch();
		}
	}
	
	public void addProcess(MOOSProcess p) {
		processes.add(p);
	}
	
	public void registerSharedVar(String varName) {
		sharedVars.add(varName);
	}
	
	public void registerSharedVars(List<String> varNames) {
		for (String v : varNames) 
			registerSharedVar(v);
	}
}
