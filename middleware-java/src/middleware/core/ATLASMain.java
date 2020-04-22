package middleware.core;

import atlasdsl.Mission;
import atlasdsl.loader.*;

public class ATLASMain {
	public static void main(String [] args) {
		// On initialisation, read the DSL concrete syntax file and construct the appropriate ATLAS objects here
		DSLLoader dslloader = new GeneratedDSLLoader();
		Mission mission;
		try {
			mission = dslloader.loadMission();
				
			MOOSATLASCore core = new MOOSATLASCore(mission);
			
			if (args[0] != null) {
			// core.setFaultDefinitionFile("fault-instance-files/test.fif");
			core.setFaultDefinitionFile(args[0]);
			}
			
			if (args[1] != null && args[1] != "false") {
				core.createGUI();
			}
				
			core.runMiddleware();
			
			
		} catch (DSLLoadFailed e) {
			System.out.println("DSL loading failed - configuration problems");
			e.printStackTrace();
		}
	}
}