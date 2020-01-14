package moosmapping.test;

import moosmapping.*;
import atlascarsgenerator.ConversionFailed;
import atlascarsgenerator.MOOSCodeGen;

import atlascollectiveintgenerator.*;

import atlasdsl.*;
import carsmapping.CARSSimulation;

public class TestMapping {
	public static void testCodeGeneration1(String code_dir) {
		MOOSSimulation testSim = new MOOSSimulation();

		MOOSCommunity gilda = new MOOSCommunity(testSim, "gilda");
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
	public static void addRobotWithSonar(Mission m, String robotName, Point startLocation, int sensorRange, double detectionProb, double falsePos, double falseNeg) {
		Robot r = new Robot(robotName);
		Sensor s = new Sensor(SensorType.SONAR);
		s.setIntComponentProperty("sensorRange", sensorRange);
		s.setIntComponentProperty("swathWidth", sensorRange);
		s.setDoubleComponentProperty("detectionProb", detectionProb);
		s.setDoubleComponentProperty("falsePos", falsePos);
		s.setDoubleComponentProperty("falseNeg", falseNeg);
		r.addSubcomponent(s);
		r.setPointComponentProperty("startLocation", startLocation);
		m.addRobot(r);
	}
	
	public static void testCodeGeneration2(String code_dir) {
		// TODO: get these consistent with the values in the report object diagram
		Mission mission = new Mission();
		Computer shoreside = new Computer("shoreside");
		mission.addComputer(shoreside);
		
		addRobotWithSonar(mission, "gilda", new Point(0.0, 0.0), 50, 0.9, 0.01, 0.05);
		addRobotWithSonar(mission, "henry", new Point(10.0, 0.0), 50, 0.8, 0.03, 0.07);
		addRobotWithSonar(mission, "frank", new Point(20.0, 0.0), 50, 0.2, 0.03, 0.02);
		addRobotWithSonar(mission, "ella",  new Point(30.0, 0.0), 50, 0.2, 0.03, 0.06);
		
		// Add objects to the environment - hazards/benign objects for the robots to find
		mission.addObject(new EnvironmentalObject(new Point(46.0, -23.0), false));
		mission.addObject(new EnvironmentalObject(new Point(36.0, -13.0), true));
		mission.addObject(new EnvironmentalObject(new Point(66.0, -3.0), false));
		
		// A test message here
		mission.addMessage(new Message("detectionGilda", mission.getRobot("gilda"), shoreside));
		
		MOOSCodeGen gen = new MOOSCodeGen(mission);
		CollectiveIntGen javaCI = new JavaCollectiveIntGen(mission);
		
		System.out.println("Converting DSL to MOOS representation...");
		try {
			CARSSimulation moossim = gen.convertDSL(mission);
			System.out.println("DSL conversion completed");
			moossim.generateCARSInterface(code_dir);
			System.out.println("Code generation completed");
			
			javaCI.generateCollectiveIntStub("/tmp/robot_ci.java", CollectiveIntGenTypes.ALL_ROBOTS);
			javaCI.generateCollectiveIntStub("/tmp/shoreside_ci.java", CollectiveIntGenTypes.ALL_COMPUTERS);
			
		} catch (ConversionFailed cf) {
			System.out.println("ERROR: DSL conversion to MOOS representation failed");
		}
	}
	
	public static void main(String [] args) {
		String outputCodeDir = System.getProperty("user.dir") + "/codegen-test1/";
		System.out.println("Generating code in: " + outputCodeDir);
		testCodeGeneration1(outputCodeDir);
		System.out.println("Code generation completed");
		
		outputCodeDir = System.getProperty("user.dir") + "/codegen-test2/";
		System.out.println("Generating code from DSL in: " + outputCodeDir);
		testCodeGeneration2(outputCodeDir);
	}
}
