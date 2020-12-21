package fuzzingengine.spec;

import java.util.Optional;
import fuzzingengine.*;
import fuzzingengine.operations.*;


public class GeneratedFuzzingSpec {
	public static FuzzingEngine createFuzzingEngine() {
	FuzzingEngine fe = new FuzzingEngine();
	FuzzingSimMapping simMapping = new FuzzingSimMapping();
	
	
	simMapping.addRecord("uSimMarine", "DESIRED_THRUST", "DEZIRED_THRUST", 
	FuzzingSimMapping.VariableDirection.INBOUND, Optional.of("/home/atlas/atlas/cust/TESTBASE"));
	simMapping.addRecord("uSimMarine", "DESIRED_RUDDER", "DEZIRED_RUDDER", 
	FuzzingSimMapping.VariableDirection.INBOUND, Optional.of("/home/atlas/atlas/cust/TESTBASE"));
		//FIR;
	fe.setSimMapping(simMapping); 
	return fe;
	}
}