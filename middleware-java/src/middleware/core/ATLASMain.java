package middleware.core;

import atlasdsl.Mission;
import atlasdsl.loader.*;

public class ATLASMain {
	public static void main(String [] args) {
		// On initialisation, read the DSL concrete syntax file and construct the appropriate ATLAS objects here
		// TODO: replace with reference to GenerateDSLLoader once the menu editor completed
		DSLLoader dslloader = new GeneratedDSLLoader();
		Mission mission;
		try {
			mission = dslloader.loadMission();
			MOOSATLASCore core = new MOOSATLASCore(mission);
			core.setFaultDefinitionFile("fault-instance-files/test.fif");
			core.runMiddleware();
		} catch (DSLLoadFailed e) {
			System.out.println("DSL loading failed - configuration problems");
			e.printStackTrace();
		}
	}
}