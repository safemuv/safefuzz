package middleware.atlascarsgenerator;
import java.util.Optional;

import atlasdsl.*;
import fuzzingengine.FuzzingEngine;
import middleware.atlascarsgenerator.carsmapping.CARSSimulation;

public abstract class CARSCodeGen {
	protected Mission mission;
	protected Optional<FuzzingEngine> fe_o; 
	
	public CARSCodeGen(Mission m, Optional<FuzzingEngine> fe_o) {
		this.mission = m;  
		this.fe_o = fe_o;
	}
	
	public abstract CARSSimulation convertDSL() throws ConversionFailed;
	
	public void generateCodeForMission(String baseDirectory) throws ConversionFailed {
		CARSSimulation c = convertDSL();
		c.generateCARSInterface(baseDirectory);
	}
}
