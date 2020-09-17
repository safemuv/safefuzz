package middleware.atlascarsgenerator;
import atlasdsl.*;
import middleware.atlascarsgenerator.carsmapping.CARSSimulation;

public abstract class CARSCodeGen {
	private Mission carsmission;
	
	public CARSCodeGen(Mission m) {
		this.carsmission = m;  
	}
	
	public abstract CARSSimulation convertDSL(Mission m) throws ConversionFailed;
	
	public void generateCodeForMission(String baseDirectory) throws ConversionFailed {
		CARSSimulation c = convertDSL(carsmission);
		c.generateCARSInterface(baseDirectory);
	}
}
