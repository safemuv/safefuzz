package faultgen.test;

import atlasdsl.Mission;
import atlasdsl.loader.DSLLoadFailed;
import atlasdsl.loader.DSLLoader;
import atlasdsl.loader.GeneratedDSLLoader;
import faultgen.*;

public class FaultFileCreationTest {
	public static void main(String [] args) {
		try {
			DSLLoader dslloader = new GeneratedDSLLoader();			
			Mission mission = dslloader.loadMission();
			FaultFileCreator ffc = new FaultFileCreator(mission, "generated-fault-instance-files");
			ffc.generateFaultListFromScratch("testrandom.fif");			
			
		} catch (DSLLoadFailed e) {
			System.out.println("DSL loading failed - configuration problems");
			e.printStackTrace();
		}
	}
}