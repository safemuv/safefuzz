package test.recorddeletion;

import java.io.FileWriter;
import java.io.IOException;
import java.util.List;

import atlasdsl.Mission;
import atlasdsl.loader.DSLLoadFailed;
import atlasdsl.loader.GeneratedDSLLoader;
import fuzzexperiment.runner.jmetal.FuzzingSelectionsSolution;
import fuzzingengine.FuzzingKeySelectionRecord;
import fuzzingengine.support.FuzzingEngineSupport;

public class TestDeleteRecords {
	public static void testRecordRemoval(Mission baseMission, String csvFile, FileWriter f) throws IOException {
		List<FuzzingKeySelectionRecord> fuzzrecs = FuzzingEngineSupport.loadFuzzingRecords(baseMission, csvFile);
		FuzzingSelectionsSolution sol = new FuzzingSelectionsSolution(baseMission, "test", false, 600.0, fuzzrecs);
		f.write("Testing file: " + csvFile + "...\n");
		f.write("\nInput: \n");
		sol.printCSVContentsToFile(f);
		sol.deleteOverlapping();
		f.write("\nOutput after deletion: \n");
		sol.printCSVContentsToFile(f);
		f.write("\n\n");
	}
	
	public static void main(String [] args) throws IOException {
		Mission baseMission;
		try {
			FileWriter f = new FileWriter("/tmp/cut.csv");
			baseMission = new GeneratedDSLLoader().loadMission();
			testRecordRemoval(baseMission, "/home/jharbin/academic/atlas/atlas-middleware/middleware-java/fuzz-configs/cutrecs/setvelocity-inner.csv", f);
			testRecordRemoval(baseMission, "/home/jharbin/academic/atlas/atlas-middleware/middleware-java/fuzz-configs/cutrecs/setvelocity-inner2.csv", f);
			testRecordRemoval(baseMission, "/home/jharbin/academic/atlas/atlas-middleware/middleware-java/fuzz-configs/cutrecs/setvelocity-nooverlap.csv", f);
			testRecordRemoval(baseMission, "/home/jharbin/academic/atlas/atlas-middleware/middleware-java/fuzz-configs/cutrecs/setvelocity-partial.csv", f);

			f.close();
		} catch (DSLLoadFailed e) {
			e.printStackTrace();
		}
	}
}
