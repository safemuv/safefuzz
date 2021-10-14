package fuzzingengine.exptgenerator;

import java.util.Optional;
import java.util.Random;

import atlasdsl.Mission;
import atlasdsl.loader.DSLLoadFailed;
import atlasdsl.loader.GeneratedDSLLoader;

public class ExperimentGeneratorTest {
	public static void main(String [] args) {
		Mission m;
		try {
			m = new GeneratedDSLLoader().loadMission();
			FuzzingTimeSpecificationGenerator tgen = new FuzzingTimeSpecificationGeneratorStartEnd(m, new Random());
			FuzzingExperimentGenerator g = new FuzzingExperimentGenerator(tgen, m);
			g.generateExperiment(Optional.of("/tmp/csvfile.fuzz"));
			System.out.println("Done");
			
		} catch (DSLLoadFailed e) {
			e.printStackTrace();
		}
	}
}
