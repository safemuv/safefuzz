package middleware.core;

import atlasdsl.Mission;
import atlasdsl.loader.*;
import middleware.logging.ATLASLog;

public class ATLASMain {
	public static void main(String [] args) {
		// On initialisation, read the DSL concrete syntax file and construct the appropriate ATLAS objects here
		DSLLoader dslloader = new GeneratedDSLLoader();
		Mission mission;
		try {
			mission = dslloader.loadMission();
			MOOSATLASCore core = new MOOSATLASCore(mission);
			// Create the GUI first, since we set its window title from the
			// fault definition file
			if (args.length > 1 && args[1] != null && (args[1].equals("nogui") == false)) {
				System.out.println("args[1] = " + args[1]);
				core.createGUI();
				ATLASLog.logMiddlewareOptions("GUI_SPEC", args[1]);
			}
			
			if (args.length > 0 && args[0] != null && (args[0].equals("nofault") == false)) {
				System.out.println("args[0] = " + args[0]);
				core.setFaultDefinitionFile(args[0]);
				ATLASLog.logMiddlewareOptions("FAULT_FILE", args[0]);
			}
			
			core.runMiddleware();
			
		} catch (DSLLoadFailed e) {
			System.out.println("DSL loading failed - configuration problems");
			e.printStackTrace();
		}
	}
}