package fuzzingengine.exptgenerator;

import java.util.Optional;
import atlasdsl.Mission;
import atlasdsl.loader.DSLLoadFailed;
import atlasdsl.loader.GeneratedDSLLoader;

public class ExperimentGeneratorTest {
	public static void main(String [] args) {
		Mission m;
		try {
			m = new GeneratedDSLLoader().loadMission();
			FuzzingExperimentGenerator g = new FuzzingExperimentFresh(m);
			g.generateExperiment(Optional.of("/tmp/csvfile.fuzz"));
			System.out.println("Done");
			
		} catch (DSLLoadFailed e) {
			e.printStackTrace();
		}
	}
}
