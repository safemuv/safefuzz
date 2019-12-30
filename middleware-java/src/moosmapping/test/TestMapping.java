package moosmapping.test;
import moosmapping.*;
import atlascarsgenerator.MOOSCodeGen;
import atlasdsl.*;
import carsmapping.CARSSimulation;

public class TestMapping {
	public static void testCodeGeneration1(String code_dir) {
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
	
	// Test function that represents a robot added with a sonar sensor
	public static void addRobotWithSonar(Mission m, String robotName, int sensorRange, double detectionProb, double falsePos, double falseNeg) {
		Robot r = new Robot(robotName);
		Sensor s = new Sensor(Sensor.SENSE_SONAR);
		s.setIntComponentProperty("sensorRange", sensorRange);
		s.setDoubleComponentProperty("detectionProb", detectionProb);
		s.setDoubleComponentProperty("falsePos", falsePos);
		s.setDoubleComponentProperty("falseNeg", falseNeg);
		r.addSubcomponent(s);
		m.addRobot(r);
	}
	
	public static void testCodeGeneration2(String code_dir) {
		// TODO: get these consistent with the values in the report object diagram
		Mission m = new Mission();
		addRobotWithSonar(m, "gilda", 50, 0.9, 0.01, 0.05);
		addRobotWithSonar(m, "henry", 50, 0.8, 0.03, 0.07);
		addRobotWithSonar(m, "frank", 50, 0.2, 0.03, 0.02);
		addRobotWithSonar(m, "ella", 50, 0.2, 0.03, 0.06);
		
		MOOSCodeGen gen = new MOOSCodeGen(m);
		System.out.println("Converting DSL to MOOS representation...");
		CARSSimulation moossim = gen.convertDSL(m);
		System.out.println("DSL conversion completed");
		moossim.generateCARSInterface(code_dir);
	}
	
	public static void main(String [] args) {
		String outputCodeDir = System.getProperty("user.dir") + "/codegen-test1/";
		System.out.println("Generating code in: " + outputCodeDir);
		testCodeGeneration1(outputCodeDir);
		System.out.println("Code generation completed");
		
		outputCodeDir = System.getProperty("user.dir") + "/codegen-test2/";
		System.out.println("Generating code from DSL in: " + outputCodeDir);
		testCodeGeneration2(outputCodeDir);
		System.out.println("Code generation completed");
	}
}
