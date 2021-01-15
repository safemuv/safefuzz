package fuzzingengine.spec;

import java.util.Optional;
import fuzzingengine.*;
import fuzzingengine.operations.*;

public class GeneratedFuzzingSpec_backup_temp {
	public static FuzzingEngine createFuzzingEngine() {
	FuzzingEngine fe = new FuzzingEngine();
	FuzzingSimMapping simMapping = new FuzzingSimMapping();
	
	//simMapping.addRecord("uSimMarine", "DESIRED_THRUST", "DEZIRED_THRUST", 
	//FuzzingSimMapping.VariableDirection.INBOUND, Optional.of("/home/atlas/atlas/cust/TESTBASE"), Optional.of("\\(([0-9]+),([0-9]+)\\)"));
	//simMapping.addRecord("uSimMarine", "DESIRED_RUDDER", "DEZIRED_RUDDER", 
	//FuzzingSimMapping.VariableDirection.INBOUND, Optional.of("/home/atlas/atlas/cust/TESTBASE"));

	
	// need to set up reflection for this key
	fe.setSimMapping(simMapping); 
	return fe;
	}
}