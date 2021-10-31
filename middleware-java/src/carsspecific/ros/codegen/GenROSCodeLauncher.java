package carsspecific.ros.codegen;

import java.util.Optional;

import atlasdsl.Mission;
import atlasdsl.loader.DSLLoadFailed;
import atlasdsl.loader.DSLLoader;
import atlasdsl.loader.GeneratedDSLLoader;
import fuzzingengine.FuzzingEngine;
import fuzzingengine.spec.GeneratedFuzzingSpec;
import middleware.atlascarsgenerator.ConversionFailed;

public class GenROSCodeLauncher {
	public static void codeGenerationROSFuzzing(String filename) throws DSLLoadFailed {
		DSLLoader dslloader = new GeneratedDSLLoader();
		Mission mission;
		mission = dslloader.loadMission();
		
		FuzzingEngine fe = GeneratedFuzzingSpec.createFuzzingEngine(mission, false);
		try {
			ROSCodeGen rgen = new ROSCodeGen(mission, Optional.of(fe), false);
			// Load the CSV file to produce fuzzing key selection records
			fe.setupFromFuzzingFile(filename, mission);
			rgen.convertDSL();
			System.out.println("Code generation completed");
		} catch (ConversionFailed cf) {
			System.out.println("ERROR: DSL conversion to MOOS representation failed: reason " + cf.getReason());
		}
	}
	
	public static void main(String [] args) {
		try {
			codeGenerationROSFuzzing("/home/jharbin/test-file-fuzzing.csv");
		}
		catch (DSLLoadFailed e) {
			System.out.println("DSL loading configuration error");
			e.printStackTrace();
			System.exit(1);
		}
	}
}