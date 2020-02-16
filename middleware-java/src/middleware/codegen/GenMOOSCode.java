package middleware.codegen;

import atlascarsgenerator.ConversionFailed;
import atlascarsgenerator.MOOSCodeGen;

import atlascollectiveintgenerator.*;

import atlasdsl.*;
import atlasdsl.loader.DSLLoader;
import atlasdsl.loader.StubDSLLoader;
import carsmapping.CARSSimulation;

public class GenMOOSCode {
	public static void testCodeGenerationMOOS(String code_dir) {
		DSLLoader dslloader = new StubDSLLoader();
		Mission mission = dslloader.loadMission();
		
		MOOSCodeGen gen = new MOOSCodeGen(mission);
		CollectiveIntGen javaCI = new JavaCollectiveIntGen(mission);		
		System.out.println("Converting DSL to MOOS representation...");
		try {
			CARSSimulation moossim = gen.convertDSL(mission);
			System.out.println("DSL conversion completed");
			moossim.generateCARSInterface(code_dir);
			System.out.println("Code generation completed");
			javaCI.generateCollectiveIntFiles("src/atlascollectiveint/custom", CollectiveIntGenTypes.ALL_COMPUTERS);
			
		} catch (ConversionFailed cf) {
			System.out.println("ERROR: DSL conversion to MOOS representation failed");
		}
	}
	
	public static void main(String [] args) {
		String outputCodeDir = System.getProperty("user.dir") + "/moos-sim/";
		System.out.println("Generating code from DSL in: " + outputCodeDir);
		testCodeGenerationMOOS(outputCodeDir);
	}
}

