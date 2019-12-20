package moosmapping.test;
import moosmapping.*;

public class TestMapping {
	public static void testCodeGeneration(String code_dir) {
		MOOSSimulation testSim = new MOOSSimulation();

		MOOSCommunity gilda = new MOOSCommunity("gilda");
		MOOSProcess ivpHelm = new MOOSProcess("pIvpHelm", gilda);
		testSim.addCommunity(gilda);
		gilda.addProcess(ivpHelm);
		
		// Add subclasses of behaviour etc with the custom
		// properties added? (for different types of behaviour)
		
		MOOSBehaviour bhv_waypoint = new MOOSBehaviour("BHV_WAYPOINT", ivpHelm);
		bhv_waypoint.setProperty("name", "waypt_return");
		ivpHelm.addBehaviour(bhv_waypoint);
		testSim.generateCARSInterface(code_dir);
	}
	
	public static void main(String [] args) {
		String outputCodeDir = System.getProperty("user.dir") + "/codegen-test/";
		System.out.println("Generating code in: " + outputCodeDir);
		testCodeGeneration(outputCodeDir);
		System.out.println("Code generation completed");
	}
}
