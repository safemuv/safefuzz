package atlascarsgenerator;
import atlasdsl.*;
import carsmapping.CARSSimulation;

public abstract class CARSCodeGen {
	private Mission carsmission;
	
	public CARSCodeGen(Mission m) {
		this.carsmission = m;  
	}
	
	public abstract CARSSimulation convertDSL(Mission m);
	
	public void generateCodeForMission(String baseDirectory) {
		CARSSimulation c = convertDSL(carsmission);
		c.generateCARSInterface(baseDirectory);
	}
}
