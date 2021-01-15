package fuzzingengine.spec;

import java.util.Optional;
import fuzzingengine.*;

public class GeneratedFuzzingSpec {
	public static FuzzingEngine createFuzzingEngine() {
	FuzzingEngine fe = new FuzzingEngine();
	FuzzingSimMapping simMapping = new FuzzingSimMapping();
	
	simMapping.addRecord("uSimMarine", "DESIRED_THRUST", "DEZIRED_THRUST", 
	FuzzingSimMapping.VariableDirection.INBOUND, Optional.of("/home/atlas/atlas/atlas-middleware/custom-moos//bin/uSimMarine"),
		Optional.empty()
	);
	simMapping.addRecord("uSimMarine", "DESIRED_RUDDER", "DEZIRED_RUDDER", 
	FuzzingSimMapping.VariableDirection.INBOUND, Optional.of("/home/atlas/atlas/atlas-middleware/custom-moos//bin/uSimMarine"),
		Optional.empty()
	);
	simMapping.addRecord("test", "TESTXXX", "TEST", 
	FuzzingSimMapping.VariableDirection.INBOUND, Optional.of("/home/atlas/atlas/atlas-middleware/custom-moos//bin/test"),
		Optional.of("\\(([0-9]+),([0-9]+)\\)")
	);
	
	fe.setSimMapping(simMapping);
	fe.setupFromFuzzingFile("/home/atlas/atlas/atlas-middleware/middleware-java/moos-sim/test.fuzz");
	return fe;
	}
}