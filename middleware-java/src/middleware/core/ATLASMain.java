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
			
			if (args.length > 0 && args[0] != null) {
				core.setFaultDefinitionFile(args[0]);
			}
			
			if (args.length > 1 && args[1] != null && args[1] != "false") {
				core.createGUI();
			}
			core.runMiddleware();
			
		} catch (DSLLoadFailed e) {
			System.out.println("DSL loading failed - configuration problems");
			e.printStackTrace();
		}
	}
}