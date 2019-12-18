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
		missionFile.write("Simulator = true");
		missionFile.write("community = " + communityName);
	}
	
	private void genANTLRBlock(FileWriter missionFile) throws IOException {
		missionFile.write("ProcessConfig = ANTLR \n{\n");
		for (MOOSProcess p : processes) {
			missionFile.write("Run = " + p.processName + "\n");
		}
		missionFile.write("}\n");
	}
	
	public void generateCode(MOOSFiles mf) {
		// Generate a base mission file
				
		String missionFileName = "targ_" + communityName + ".moos";
		try {
			FileWriter missionFile = mf.getOpenFile(missionFileName);
			genBasicMissionParams(missionFile);
			genANTLRBlock(missionFile);
			
			for (MOOSProcess p : processes) {
				p.generateConfigBlock(missionFile);
			}
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
