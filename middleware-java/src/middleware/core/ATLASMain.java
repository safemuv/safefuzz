package middleware.core;

import atlasdsl.Mission;
import atlasdsl.loader.DSLLoader;
import atlasdsl.loader.StubDSLLoader;

public class ATLASMain {
	public static void main(String [] args) {
		// On initialisation, read the DSL concrete syntax file and construct the appropriate ATLAS objects here
		// TODO: replace with a real loader once the concrete syntax is ready
		DSLLoader dslloader = new StubDSLLoader();
		Mission mission = dslloader.loadMission();
		
		MOOSATLASCore core = new MOOSATLASCore(mission);
		core.runMiddleware();
	}
}