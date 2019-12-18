package moosmapping.test;
import moosmapping.*;

public class TestMapping {
	public static void testCodeGeneration(String code_dir) {
		MOOSSimulation testSim = new MOOSSimulation();

		MOOSCommunity gilda = new MOOSCommunity("gilda");
		MOOSProcess ivpHelm = new MOOSProcess("ivp-helm", gilda);
		testSim.addCommunity(gilda);
		gilda.addProcess(ivpHelm);
		
		MOOSBehaviour bhv_waypoint = new MOOSBehaviour();
		ivpHelm.addBehaviour(bhv_waypoint);
		testSim.generateCARSInterface(code_dir);
	}
	
	public static void main(String [] args) {
		testCodeGeneration("/tmp/testdir/");
		System.out.println("Code generation completed");
	}
}
